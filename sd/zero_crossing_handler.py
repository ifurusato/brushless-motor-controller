#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-09
# modified: 2025-07-09

import utime
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from motor import Motor
from zc_stage import ZeroCrossingStage
from fsm import FiniteStateMachine

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class IllegalStateError(RuntimeError):
    '''
    Exception raised when an invalid state transition is attempted.
    '''
    pass

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ZeroCrossingHandler(FiniteStateMachine):
    '''
    Manages the zero-crossing transition for a single motor.
    '''
    def __init__(self, motor_id, config, motor_instance, level=Level.INFO):
        self._log = Logger('zc-handler-{}'.format(motor_id), level=level)
        self._log.info(Fore.YELLOW + 'initializing ZC Handler for motor {}'.format(motor_id))
        self._motor = motor_instance
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['kros']['zero_crossing_handler']
        self._decel_rate_rpm_per_sec    = _cfg.get('decel_rate_rpm_per_sec', 300)
        self._accel_rate_rpm_per_sec    = _cfg.get('accel_rate_rpm_per_sec', 300)
        self._transition_interval_ms    = _cfg.get('transition_interval_ms', 20)
        self._stop_rpm_threshold        = _cfg.get('stop_rpm_threshold', 5.0)
        self._zero_confirmation_time_ms = _cfg.get('confirmation_time_ms', 100)
        self._max_transition_time_ms    = _cfg.get('max_transition_time_ms', 5000)
        self._log.info(Fore.YELLOW + 'ZC for motor {}: decel_rate={:>5.1f} RPM/s, accel_rate={:>5.1f} RPM/s, interval={}ms'.format(
                motor_id, self._decel_rate_rpm_per_sec, self._accel_rate_rpm_per_sec, self._transition_interval_ms))
        self._log.info(Fore.YELLOW + 'ZC for motor {}: stop_threshold={:>5.1f} RPM, confirm_time={}ms, max_time={}ms'.format(
                motor_id, self._stop_rpm_threshold, self._zero_confirmation_time_ms, self._max_transition_time_ms))
        # define ZC state transitions for the FSM
        _zc_transition_map = {
            ZeroCrossingStage.IDLE:            {ZeroCrossingStage.DECELERATING, ZeroCrossingStage.IDLE},
            ZeroCrossingStage.DECELERATING:    {ZeroCrossingStage.CONFIRMING_STOP, ZeroCrossingStage.IDLE},
            ZeroCrossingStage.CONFIRMING_STOP: {ZeroCrossingStage.ACCELERATING, ZeroCrossingStage.IDLE},
            ZeroCrossingStage.ACCELERATING:    {ZeroCrossingStage.COMPLETE, ZeroCrossingStage.IDLE},
            ZeroCrossingStage.COMPLETE:        {ZeroCrossingStage.IDLE},
        }
        # initialize the FSM. This sets self._state to ZeroCrossingStage.IDLE initially.
        super().__init__(
            logger=self._log,
            task_name='zc-fsm-{}'.format(motor_id),
            state_class=ZeroCrossingStage,
            initial_state=ZeroCrossingStage.IDLE,
            transition_map=_zc_transition_map
        )
        self._is_active        = False
        self._current_zc_task  = None
        # self._current_zc_stage is managed by the FSM's self._state
        self._effective_pid_target_rpm          = 0.0
        self._initial_motor_rpm_for_transition  = 0.0
        self._final_target_rpm_after_transition = 0.0
        # register state entry callback methods
        self.on(ZeroCrossingStage.DECELERATING, self._on_decelerating_entry)
        self.on(ZeroCrossingStage.CONFIRMING_STOP, self._on_confirming_stop_entry)
        self.on(ZeroCrossingStage.ACCELERATING, self._on_accelerating_entry)
        self.on(ZeroCrossingStage.COMPLETE, self._on_complete_entry)
        self.on(ZeroCrossingStage.IDLE, self._on_idle_entry)
        self._log.info(Fore.YELLOW + 'ready.')

    @property
    def is_active(self):
        '''
        Returns True if a zero-crossing operation is in progress.
        This property now relies on the internal `_is_active` flag,
        which is managed by task lifecycle and FSM state transitions.
        '''
        return self._is_active

    @property
    def get_effective_pid_target_rpm(self):
        '''
        Returns the current ZCH-generated target RPM for the PID controller.
        '''
        return self._effective_pid_target_rpm

    def _on_idle_entry(self):
        '''
        Callback executed when entering the IDLE state.
        '''
        self._log.info(Fore.YELLOW + 'motor {}: FSM entered IDLE stage.'.format(self._motor.name))
        self._is_active = False # Ensure active flag is False
        self._effective_pid_target_rpm = 0.0 # Ensure target is 0 when idle

    def _on_decelerating_entry(self):
        '''
        Callback executed when entering the DECELERATING state.
        '''
        self._log.info(Fore.YELLOW + 'motor {}: FSM entered DECELERATING stage.'.format(self._motor.name))

    def _on_confirming_stop_entry(self):
        '''
        Callback executed when entering the CONFIRMING_STOP state.
        '''
        self._log.info(Fore.YELLOW + 'motor {}: FSM entered CONFIRMING_STOP stage.'.format(self._motor.name))
        self._effective_pid_target_rpm = 0.0 # Ensure target is zero for confirmation
        self._stop_confirmation_start_time = utime.ticks_ms() # Initialize timer for this stage

    def _on_accelerating_entry(self):
        '''
        Callback executed when entering the ACCELERATING state.
        '''
        self._log.info(Fore.YELLOW + 'motor {}: FSM entered ACCELERATING stage.'.format(self._motor.name))

    def _on_complete_entry(self):
        '''
        Callback executed when entering the COMPLETE state.
        '''
        self._log.info(Fore.YELLOW + 'motor {}: FSM entered COMPLETE stage.'.format(self._motor.name))
        self._is_active = False # Mark ZC handler as inactive
        self._effective_pid_target_rpm = self._final_target_rpm_after_transition # Ensure final target is precisely set

    async def _run_transition_task(self):
        '''
        Internal asynchronous task to manage the zero-crossing sequence.
        This task now drives the FSM transitions.
        '''
        try:
            self._is_active = True # Set overall active flag at task start

            # These values are set once at the beginning of the transition
            # and used across all stages.
            decel_step = (self._decel_rate_rpm_per_sec / 1000) * self._transition_interval_ms
            accel_step = (self._accel_rate_rpm_per_sec / 1000) * self._transition_interval_ms
            intended_logical_direction = Motor.DIRECTION_FORWARD if self._final_target_rpm_after_transition >= 0 else Motor.DIRECTION_REVERSE

            self._log.info(Fore.YELLOW + 'motor {}: ZC transition task started; initial RPM: {:.1f}; final target: {:.1f}; triggering DECELERATING stage…'.format(
                    self._motor.name, self._initial_motor_rpm_for_transition, self._final_target_rpm_after_transition))
            # the first transition to DECELERATING will trigger its '_on_decelerating_entry' callback
            self.transition(ZeroCrossingStage.DECELERATING)

            start_transition_time_ms = utime.ticks_ms() # overall timeout for the entire ZC process

            # Phase 1: Decelerate to Zero RPM
            while self.state == ZeroCrossingStage.DECELERATING and \
                  utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms:

                if self._effective_pid_target_rpm > 0:
                    self._effective_pid_target_rpm = max(0.0, self._effective_pid_target_rpm - decel_step)
                else:
                    self._effective_pid_target_rpm = min(0.0, self._effective_pid_target_rpm + decel_step)

                self._log.debug('motor {}: decelerating PID target to {:.1f} RPM; actual RPM: {:.1f}'.format(
                    self._motor.name, self._effective_pid_target_rpm, self._motor.rpm))

                if abs(self._motor.rpm) <= self._stop_rpm_threshold and abs(self._effective_pid_target_rpm) < self._stop_rpm_threshold:
                    self.transition(ZeroCrossingStage.CONFIRMING_STOP) # This triggers _on_confirming_stop_entry
                    break

                await asyncio.sleep_ms(self._transition_interval_ms)

            # if still in DECELERATING state after the loop, it means timeout occurred
            if self.state == ZeroCrossingStage.DECELERATING:
                self._log.warning(Fore.YELLOW + 'motor {}: deceleration timed out; forcing target to 0 RPM and proceeding…'.format(self._motor.name))
                self._effective_pid_target_rpm = 0.0 # Ensure target is zero
                self.transition(ZeroCrossingStage.CONFIRMING_STOP) # This triggers _on_confirming_stop_entry

            # Phase 2: Confirm Physical Stop and Set Direction
            if self.state == ZeroCrossingStage.CONFIRMING_STOP: # Check current FSM state
                is_physically_stopped = False
                while self.state == ZeroCrossingStage.CONFIRMING_STOP and not is_physically_stopped and \
                      utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms: # Use overall timeout
                    current_rpm_magnitude = abs(self._motor.rpm)
                    if current_rpm_magnitude <= self._stop_rpm_threshold:
                        # Check against _stop_confirmation_start_time which is set in _on_confirming_stop_entry
                        if utime.ticks_diff(utime.ticks_ms(), self._stop_confirmation_start_time) >= self._zero_confirmation_time_ms:
                            is_physically_stopped = True
                            self._log.info(Fore.YELLOW + 'motor {}: confirmed stopped ({:.1f} RPM <= {:.1f} RPM) for {}ms.'.format(
                                self._motor.name, current_rpm_magnitude, self._stop_rpm_threshold, self._zero_confirmation_time_ms))
                    else:
                        self._stop_confirmation_start_time = utime.ticks_ms() # Reset timer if motor moves again
                        self._log.debug('motor {}: confirming stop… current RPM: {:.1f}'.format(self._motor.name, current_rpm_magnitude))
                    await asyncio.sleep_ms(self._transition_interval_ms)

                if not is_physically_stopped:
                    self._log.warning(Fore.YELLOW + 'motor {}: physical stop confirmation timed out (current RPM: {:.1f}); proceeding with direction change…'.format(
                        self._motor.name, abs(self._motor.rpm)))

                # set motor direction here, before accelerating in the new direction
                self._motor.direction = intended_logical_direction
                self._log.info(Fore.YELLOW + 'motor {}: Direction set to {}'.format(self._motor.name, 'FORWARD' if intended_logical_direction else 'REVERSE'))
                self.transition(ZeroCrossingStage.ACCELERATING) # This triggers _on_accelerating_entry

            # Phase 3: Accelerate in New Direction
            while self.state == ZeroCrossingStage.ACCELERATING \
                    and utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms:
                if self._final_target_rpm_after_transition > 0:
                    self._effective_pid_target_rpm = min(self._final_target_rpm_after_transition, self._effective_pid_target_rpm + accel_step)
                else:
                    self._effective_pid_target_rpm = max(self._final_target_rpm_after_transition, self._effective_pid_target_rpm - accel_step)
                self._log.debug('motor {}: accelerating PID target to {:.1f} RPM; actual RPM: {:.1f}'.format(
                        self._motor.name, self._effective_pid_target_rpm, self._motor.rpm))
                if (self._final_target_rpm_after_transition >= 0 and self._effective_pid_target_rpm >= self._final_target_rpm_after_transition) or \
                   (self._final_target_rpm_after_transition < 0 and self._effective_pid_target_rpm <= self._final_target_rpm_after_transition):
                    self._log.info(Fore.YELLOW + 'motor {}: reached final target RPM {:.1f}; transition complete.'.format(self._motor.name, self._final_target_rpm_after_transition))
                    self.transition(ZeroCrossingStage.COMPLETE) # This triggers _on_complete_entry
                    break

            # if still in ACCELERATING state after the loop, it means timeout occurred or target not precisely reached
            if self.state == ZeroCrossingStage.ACCELERATING:
                self._log.warning(Fore.YELLOW + 'motor {}: acceleration timed out or target not precisely reached. Forcing to COMPLETE.'.format(self._motor.name))
                self.transition(ZeroCrossingStage.COMPLETE) # this triggers _on_complete_entry
            # final state handling after the loops
            self._log.info(Fore.YELLOW + 'motor {}: zero-crossing process completed.'.format(self._motor.name))
            return True # indicate successful completion of the internal task

        except asyncio.CancelledError:
            self._log.info(Fore.YELLOW + 'motor {}: ZC task cancelled; resetting state.'.format(self._motor.name))
            self.reset()
            raise
        except Exception as e:
            self._log.error(Fore.RED + 'Error in ZC handler for motor {}: {}'.format(self._motor.name, e))
            self.reset()
            raise

    def _start_zero_crossing(self, initial_actual_rpm, new_final_target_rpm_signed):
        '''
        Initiates the zero-crossing process.
        '''
        if self._is_active:
            self._log.warning(Fore.YELLOW + 'motor {}: ZC already active; ignoring new request.'.format(self._motor.name))
            return False

        self._initial_motor_rpm_for_transition = initial_actual_rpm
        self._final_target_rpm_after_transition = new_final_target_rpm_signed
        self._current_zc_task = asyncio.create_task(self._run_transition_task())
        return True

    def cancel_zero_crossing(self):
        '''
        Cancels an ongoing zero-crossing operation.
        '''
        if self._current_zc_task and not self._current_zc_task.done():
            self._current_zc_task.cancel()
            self._log.info(Fore.YELLOW + 'motor {}: ZC task explicitly cancelled.'.format(self._motor.name))
        else:
            self._log.debug('motor {}: No active ZC task to cancel.'.format(self._motor.name))
        self.reset() # This will transition FSM to IDLE via reset() -> _on_idle_entry
        # ensure the motor is immediately stopped if ZC is cancelled
        self._motor.stop()

    def reset(self):
        '''
        Resets the internal state of the ZC handler.
        This also transitions the FSM back to IDLE.
        '''
        self._is_active = False # Set inactive
        self.transition(ZeroCrossingStage.IDLE) # Use FSM transition to reset state, triggers _on_idle_entry
        self._effective_pid_target_rpm = 0.0
        self._initial_motor_rpm_for_transition = 0.0
        self._final_target_rpm_after_transition = 0.0
        if self._current_zc_task:
            if not self._current_zc_task.done():
                self._current_zc_task.cancel()
            self._current_zc_task = None
        self._log.info(Fore.YELLOW + 'zero crossing handler reset.')
        # ensure the motor is immediately stopped when reset
        self._motor.stop()

    def handle_new_command(self, commanded_target_rpm, current_actual_motor_rpm, pid_controller_instance):
        '''
        Processes a new commanded target RPM and manages the ZC state accordingly.
        This method encapsulates all ZC-related decision-making.

        Args:
            commanded_target_rpm (float): The new RPM commanded by the higher-level controller.
            current_actual_motor_rpm (float): The current measured RPM of the motor.
            pid_controller_instance (PIDController): The PID controller instance for this motor.

        Returns:
            bool: True if a new ZC transition task was initiated or restarted,
                  False otherwise (meaning ZCH is now idle or already idle).
        '''
        new_command_direction = Motor.DIRECTION_FORWARD if commanded_target_rpm >= 0 else Motor.DIRECTION_REVERSE
        current_actual_motor_direction = Motor.DIRECTION_FORWARD if current_actual_motor_rpm >= 0 else Motor.DIRECTION_REVERSE
        transition_initiated_or_restarted = False

        if commanded_target_rpm == 0.0:
            # Case A: An explicit STOP command (0.0 RPM)
            self._log.debug(Fore.YELLOW + 'Case A: An explicit STOP command (0.0 RPM)')
            if self.is_active:
                self._log.info(Fore.YELLOW + 'motor {}: ZC active but new command is STOP (0 RPM); cancelling ZC…'.format(self._motor.name))
                self.cancel_zero_crossing() # This will transition FSM to IDLE via reset() -> _on_idle_entry
                pid_controller_instance.reset() # Reset PID after cancellation
            # No transition initiated here, ZCH is now idle.

        elif self.is_active:
            # Case B: New command arrives while ZC is active (but not a STOP)
            self._log.info(Fore.YELLOW + 'Case B: New command arrives while ZC is active (but not a STOP)')
            # Check if the new command effectively cancels the need for the current reversal
            if new_command_direction == current_actual_motor_direction:
                self._log.info(Fore.YELLOW + 'motor {}: ZC active but new target ({:.1f} RPM) is in current actual direction ({:.1f} RPM); cancelling ZC…'.format(
                    self._motor.name, commanded_target_rpm, current_actual_motor_rpm))
                self.cancel_zero_crossing() # ZC is no longer active
                pid_controller_instance.reset() # Reset PID after cancellation

            # Check if the new command represents a *different* reversal profile
            elif self._final_target_rpm_after_transition != float(commanded_target_rpm):
                self._log.info(Fore.YELLOW + 'motor {}: ZC active, but new command requires new reversal profile '.format(self._motor.name)
                        + Style.DIM + '(old final {:.1f} RPM -> new final {:.1f} RPM); cancelling old ZC and starting new…'.format(self._final_target_rpm_after_transition, commanded_target_rpm))
                # immediately cancel the old ZC task (sets is_active=False, transitions FSM to IDLE)
                self.cancel_zero_crossing()
                pid_controller_instance.reset() # Reset PID
                # now, start a new ZC, which will set is_active=True and launch a new task
                self._start_zero_crossing(current_actual_motor_rpm, float(commanded_target_rpm))
                transition_initiated_or_restarted = True
            else:
                # ZC is active, and the new command is identical to the current ZC's final target.
                # Do nothing, let the current ZC complete naturally.
                self._log.debug('motor {}: ZC active and new command ({:.1f} RPM) is consistent with current ZC; deferring…'.format(self._motor.name, commanded_target_rpm))
                transition_initiated_or_restarted = True # ZCH remains responsible for this transition

        else:
            # Case C: ZC is not active (normal PID operation or needs to start ZC)
            self._log.debug(Fore.YELLOW + Style.DIM + 'Case C: ZC is not active (normal PID operation or needs to start ZC)')
            # Check if this new command should *initiate* a zero-crossing
            if current_actual_motor_direction != new_command_direction and \
               abs(current_actual_motor_rpm) > self._motor._soft_stop_threshold_rpm:
                self._log.info(Fore.YELLOW + 'motor {}: direction reversal detected (actual dir {} -> new command dir {}); actual RPM {:.1f}; initiating ZC…'.format(
                    self._motor.name, 'F' if current_actual_motor_direction else 'R', 'F' if new_command_direction else 'R', current_actual_motor_rpm))
                self._start_zero_crossing(current_actual_motor_rpm, float(commanded_target_rpm))
                transition_initiated_or_restarted = True
            # else: No ZC needed, ZCH remains inactive.

        return transition_initiated_or_restarted

#EOF
