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

import uasyncio as asyncio
import utime
from logger import Logger, Level
from colorama import Fore, Style
from micropython import const
from motor import Motor

# Define ZC stages
_ZC_STAGE_DECELERATING    = const(0)
_ZC_STAGE_CONFIRMING_STOP = const(1)
_ZC_STAGE_ACCELERATING    = const(2)
_ZC_STAGE_COMPLETE        = const(3)
_ZC_STAGE_IDLE            = const(4)

class ZeroCrossingHandler:
    '''
    Manages the zero-crossing transition for a single motor.
    '''
    def __init__(self, motor_id, config, motor_instance, level=Level.INFO):
        self._log = Logger('zc-handler-{}'.format(motor_id), level=level)
        self._log.info(Fore.YELLOW + 'initializing ZC Handler for motor {}'.format(motor_id))
        _cfg = config['kros']['zero_crossing_handler']
        self._motor = motor_instance

        self._decel_rate_rpm_per_sec    = _cfg.get('decel_rate_rpm_per_sec', 300)
        self._accel_rate_rpm_per_sec    = _cfg.get('accel_rate_rpm_per_sec', 300)
        self._transition_interval_ms    = _cfg.get('transition_interval_ms', 20)
        self._stop_rpm_threshold        = _cfg.get('stop_rpm_threshold', 5.0)
        self._zero_confirmation_time_ms = _cfg.get('confirmation_time_ms', 100)
        self._max_transition_time_ms    = _cfg.get('max_transition_time_ms', 5000)

        self._log.info(Fore.YELLOW + "ZC for motor {}: decel_rate={:>5.1f} RPM/s, accel_rate={:>5.1f} RPM/s, interval={}ms".format(
                motor_id, self._decel_rate_rpm_per_sec, self._accel_rate_rpm_per_sec, self._transition_interval_ms))
        self._log.info(Fore.YELLOW + "ZC for motor {}: stop_threshold={:>5.1f} RPM, confirm_time={}ms, max_time={}ms".format(
                motor_id, self._stop_rpm_threshold, self._zero_confirmation_time_ms, self._max_transition_time_ms))
        
        self._is_active        = False
        self._current_zc_task  = None
        self._current_zc_stage = _ZC_STAGE_IDLE
        self._effective_pid_target_rpm          = 0.0
        self._initial_motor_rpm_for_transition  = 0.0
        self._final_target_rpm_after_transition = 0.0
        self._log.info(Fore.YELLOW + 'ready.')

    @property
    def is_active(self):
        '''
        Returns True if a zero-crossing operation is in progress.
        '''
        return self._is_active

    @property
    def get_effective_pid_target_rpm(self):
        '''
        Returns the current ZCH-generated target RPM for the PID controller.
        '''
        return self._effective_pid_target_rpm

    async def _run_transition_task(self):
        '''
        Internal asynchronous task to manage the zero-crossing sequence.
        '''
        self._log.info(Fore.YELLOW + '_run_transition_task begin. ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈ ')
        try:
            self._is_active = True
            self._current_zc_stage = _ZC_STAGE_DECELERATING
            self._effective_pid_target_rpm = self._initial_motor_rpm_for_transition
            
            decel_step = (self._decel_rate_rpm_per_sec / 1000) * self._transition_interval_ms
            accel_step = (self._accel_rate_rpm_per_sec / 1000) * self._transition_interval_ms
            intended_logical_direction = Motor.DIRECTION_FORWARD if self._final_target_rpm_after_transition >= 0 else Motor.DIRECTION_REVERSE
            
            self._log.info(Fore.YELLOW + "motor {}: ZC task started. Initial RPM: {:.1f}, Final target: {:.1f}. Stage: Decelerating.".format(
                self._motor.name, self._initial_motor_rpm_for_transition, self._final_target_rpm_after_transition))

            start_transition_time_ms = utime.ticks_ms()

            # Phase 1: Decelerate to Zero RPM
            while self._is_active and self._current_zc_stage == _ZC_STAGE_DECELERATING and \
                  utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms:
                
                if self._effective_pid_target_rpm > 0:
                    self._effective_pid_target_rpm = max(0.0, self._effective_pid_target_rpm - decel_step)
                else:
                    self._effective_pid_target_rpm = min(0.0, self._effective_pid_target_rpm + decel_step)

                self._log.debug("Motor {}: Decel PID target to {:.1f} RPM. Actual RPM: {:.1f}".format(
                    self._motor.name, self._effective_pid_target_rpm, self._motor.rpm))

                if abs(self._motor.rpm) <= self._stop_rpm_threshold and abs(self._effective_pid_target_rpm) < self._stop_rpm_threshold:
                    self._current_zc_stage = _ZC_STAGE_CONFIRMING_STOP
                    self._effective_pid_target_rpm = 0.0
                    self._log.info(Fore.YELLOW + "motor {}: Reached zero RPM target. Confirming Stop.".format(self._motor.name))
                    break

                await asyncio.sleep_ms(self._transition_interval_ms)
            
            if self._current_zc_stage == _ZC_STAGE_DECELERATING:
                self._log.warning(Fore.YELLOW + "motor {}: Deceleration timed out. Forcing target to 0 RPM and proceeding.".format(self._motor.name))
                self._effective_pid_target_rpm = 0.0
                self._current_zc_stage = _ZC_STAGE_CONFIRMING_STOP


            # Phase 2: Confirm Physical Stop and Set Direction
            if self._is_active and self._current_zc_stage == _ZC_STAGE_CONFIRMING_STOP:
                stop_confirmation_start_time = utime.ticks_ms()
                is_physically_stopped = False

                while self._is_active and not is_physically_stopped and \
                      utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms:
                    
                    current_rpm_magnitude = abs(self._motor.rpm)
                    
                    if current_rpm_magnitude <= self._stop_rpm_threshold:
                        if utime.ticks_diff(utime.ticks_ms(), stop_confirmation_start_time) >= self._zero_confirmation_time_ms:
                            is_physically_stopped = True
                            self._log.info(Fore.YELLOW + "motor {}: Confirmed stopped ({} RPM <= {} RPM) for {}ms.".format(
                                self._motor.name, current_rpm_magnitude, self._stop_rpm_threshold, self._zero_confirmation_time_ms))
                    else:
                        stop_confirmation_start_time = utime.ticks_ms()
                        self._log.debug("Motor {}: Confirming stop... Current RPM: {:.1f}".format(self._motor.name, current_rpm_magnitude))

                    await asyncio.sleep_ms(self._transition_interval_ms)

                if not is_physically_stopped:
                    self._log.warning(Fore.YELLOW + "motor {}: Physical stop confirmation timed out (current RPM: {:.1f}). Proceeding with direction change.".format(
                        self._motor.name, abs(self._motor.rpm)))
                
                self._motor.direction = intended_logical_direction
                self._log.info(Fore.YELLOW + "motor {}: Direction set to {}".format(self._motor.name, "FORWARD" if intended_logical_direction else "REVERSE"))
                self._current_zc_stage = _ZC_STAGE_ACCELERATING


            # Phase 3: Accelerate in New Direction
            while self._is_active and self._current_zc_stage == _ZC_STAGE_ACCELERATING and \
                  utime.ticks_diff(utime.ticks_ms(), start_transition_time_ms) < self._max_transition_time_ms:
                
                if self._final_target_rpm_after_transition > 0:
                    self._effective_pid_target_rpm = min(self._final_target_rpm_after_transition, self._effective_pid_target_rpm + accel_step)
                else:
                    self._effective_pid_target_rpm = max(self._final_target_rpm_after_transition, self._effective_pid_target_rpm - accel_step)

                self._log.debug("Motor {}: Accel PID target to {:.1f} RPM. Actual RPM: {:.1f}".format(
                    self._motor.name, self._effective_pid_target_rpm, self._motor.rpm))

                if (self._final_target_rpm_after_transition >= 0 and self._effective_pid_target_rpm >= self._final_target_rpm_after_transition) or \
                   (self._final_target_rpm_after_transition < 0 and self._effective_pid_target_rpm <= self._final_target_rpm_after_transition):
                    self._log.info(Fore.YELLOW + "motor {}: Reached final target RPM {:.1f}. Transition complete.".format(self._motor.name, self._final_target_rpm_after_transition))
                    self._current_zc_stage = _ZC_STAGE_COMPLETE
                    break

            self._current_zc_stage = _ZC_STAGE_COMPLETE
            self._is_active = False
            self._log.info(Fore.YELLOW + "motor {}: zero-crossing process completed.".format(self._motor.name))
            return True # Indicate successful completion of the internal task
            
        except asyncio.CancelledError:
            self._log.info(Fore.YELLOW + "motor {}: ZC task cancelled. Resetting state.".format(self._motor.name))
            self.reset()
            raise
        except Exception as e:
            self._log.error(Fore.RED + "Error in ZC handler for motor {}: {}".format(self._motor.name, e))
            self.reset()
            raise

    def start_zero_crossing(self, initial_actual_rpm, new_final_target_rpm_signed):
        """Initiates the zero-crossing process."""
        if self._is_active:
            self._log.warning(Fore.YELLOW + "motor {}: ZC already active. Ignoring new request.".format(self._motor.name))
            return False

        self._initial_motor_rpm_for_transition = initial_actual_rpm
        self._final_target_rpm_after_transition = new_final_target_rpm_signed
        self._current_zc_task = asyncio.create_task(self._run_transition_task())
        return True

    def cancel_zero_crossing(self):
        """Cancels an ongoing zero-crossing operation."""
        if self._current_zc_task and not self._current_zc_task.done():
            self._current_zc_task.cancel()
            self._log.info(Fore.YELLOW + "Motor {}: ZC task explicitly cancelled.".format(self._motor.name))
        else:
            self._log.debug("Motor {}: No active ZC task to cancel.".format(self._motor.name))
        self.reset()
        # ensure the motor is immediately stopped if ZC is cancelled
        self._motor.stop() # ADD THIS LINE

    def reset(self):
        """Resets the internal state of the ZC handler."""
        self._is_active = False
        self._current_zc_stage = _ZC_STAGE_IDLE
        self._effective_pid_target_rpm = 0.0 
        self._initial_motor_rpm_for_transition = 0.0
        self._final_target_rpm_after_transition = 0.0
        if self._current_zc_task:
            if not self._current_zc_task.done():
                self._current_zc_task.cancel()
            self._current_zc_task = None
        self._log.info(Fore.YELLOW + "zero crossing handler reset.")
        # ensure the motor is immediately stopped when reset
        self._motor.stop() # ADD THIS LINE

    def handle_new_command(self, commanded_target_rpm, current_actual_motor_rpm, pid_controller_instance):
        """
        Processes a new commanded target RPM and manages the ZC state accordingly.
        This method encapsulates all ZC-related decision-making.

        Args:
            commanded_target_rpm (float): The new RPM commanded by the higher-level controller.
            current_actual_motor_rpm (float): The current measured RPM of the motor.
            pid_controller_instance (PIDController): The PID controller instance for this motor.

        Returns:
            bool: True if a new ZC transition task was initiated or restarted,
                  False otherwise (meaning ZCH is now idle or already idle).
        """
        new_command_direction = Motor.DIRECTION_FORWARD if commanded_target_rpm >= 0 else Motor.DIRECTION_REVERSE
        current_actual_motor_direction = Motor.DIRECTION_FORWARD if current_actual_motor_rpm >= 0 else Motor.DIRECTION_REVERSE
        
        transition_initiated_or_restarted = False

        # Case A: An explicit STOP command (0.0 RPM)
        if commanded_target_rpm == 0.0:
            self._log.info(Fore.YELLOW + 'Case A: An explicit STOP command (0.0 RPM)')
            if self.is_active:
                self._log.info(Fore.YELLOW + "motor {}: ZC active but new command is STOP (0 RPM). Cancelling ZC.".format(self._motor.name))
                self.cancel_zero_crossing() # This will set _effective_pid_target_rpm to 0.0, is_active to False
                pid_controller_instance.reset() # Reset PID after cancellation
            # no transition initiated here, ZCH is now idle.
            
        # Case B: New command arrives while ZC is active (but not a STOP)
        elif self.is_active:
            self._log.info(Fore.YELLOW + 'Case B: New command arrives while ZC is active (but not a STOP)')
            # Check if the new command effectively cancels the need for the current reversal
            if new_command_direction == current_actual_motor_direction:
                self._log.info(Fore.YELLOW + "motor {}: ZC active but new target ({:.1f} RPM) is in current actual direction ({:.1f} RPM). Cancelling ZC.".format(
                    self._motor.name, commanded_target_rpm, current_actual_motor_rpm))
                self.cancel_zero_crossing() # ZC is no longer active
                pid_controller_instance.reset() # Reset PID after cancellation
            
            # Check if the new command represents a *different* reversal profile
            elif self._final_target_rpm_after_transition != float(commanded_target_rpm):
                self._log.info(Fore.YELLOW + "motor {}: ZC active, but new command requires *new* reversal profile (old final {:.1f} RPM -> new final {:.1f} RPM). Cancelling old ZC and starting new.".format(
                    self._motor.name, self._final_target_rpm_after_transition, commanded_target_rpm))
                
                # Immediately cancel the old ZC task (sets is_active=False, resets effective target to 0)
                self.cancel_zero_crossing() 
                pid_controller_instance.reset() # Reset PID
                
                # Now, start a new ZC, which will set is_active=True and launch a new task
                self.start_zero_crossing(current_actual_motor_rpm, float(commanded_target_rpm))
                transition_initiated_or_restarted = True
            else:
                # ZC is active, and the new command is identical to the current ZC's final target.
                # Do nothing, let the current ZC complete naturally.
                self._log.debug("Motor {}: ZC active and new command ({:.1f} RPM) is consistent with current ZC. Deferring.".format(self._motor.name, commanded_target_rpm))
                transition_initiated_or_restarted = True # ZCH remains responsible for this transition

        # Case C: ZC is not active (normal PID operation or needs to start ZC)
        else:
            self._log.info(Fore.YELLOW + Style.DIM + 'Case C: ZC is not active (normal PID operation or needs to start ZC)')
            # Check if this new command should *initiate* a zero-crossing
            if current_actual_motor_direction != new_command_direction and \
               abs(current_actual_motor_rpm) > self._motor._soft_stop_threshold_rpm: 
                
                self._log.info(Fore.YELLOW + "motor {}: Direction reversal detected (actual dir {} -> new command dir {}). Actual RPM {:.1f}. Initiating ZC.".format(
                    self._motor.name, "F" if current_actual_motor_direction else "R", "F" if new_command_direction else "R", current_actual_motor_rpm))
                
                self.start_zero_crossing(current_actual_motor_rpm, float(commanded_target_rpm))
                transition_initiated_or_restarted = True
            # else: No ZC needed, ZCH remains inactive.
        
        return transition_initiated_or_restarted

#EOF
