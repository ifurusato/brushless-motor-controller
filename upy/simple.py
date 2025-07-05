
import time
import utime # Added for microsecond timing for encoder
from pyb import Pin, ExtInt, Timer

# --- Global/Shared State for the Encoder Interrupt Service Routine (ISR) ---
# We use a list to store mutable state that the ISR can modify and the main loop can read.
# [0] = tick_count (increments on each valid encoder pulse)
# [1] = last_valid_pulse_time_us (timestamp of the last valid encoder pulse)
# [2] = last_calculated_interval_us (the time between the last two valid pulses)
encoder_state = [0, 0, 0] # Initialize tick_count, last_pulse_time, and interval
DEBOUNCE_PERIOD_US = 500 # 0.5 milliseconds debounce period. Adjust if needed.

# --- Encoder Callback (ISR) ---
# This function is called every time a falling edge is detected on the encoder pin.
def _simple_encoder_callback(pin):
    global counter, encoder_state 
    current_time_us = utime.ticks_us()
    
    # Calculate the raw time difference between this pulse and the last valid one
    raw_interval_us = utime.ticks_diff(current_time_us, encoder_state[1])

    # Apply software debounce: only process if enough time has passed
    if raw_interval_us >= DEBOUNCE_PERIOD_US:
        encoder_state[0] += 1          # Increment the tick count
        encoder_state[1] = current_time_us # Update the timestamp of the last valid pulse
        encoder_state[2] = raw_interval_us # Store the measured interval for external use

def main():
    print("start…")
    # --- Pin Definitions ---
    M0 = 'PB6' # PWM pin for Motor 0
    M1 = 'PB7'
    M2 = 'PB8'
    M3 = 'PB9'
    D0 = 'PB12' # Direction pin for Motor 0
    D1 = 'PB14'
    D2 = 'PB3'
    D3 = 'PB5'
    ENCODER_PIN_NAME = 'PC6' # Encoder pin for Motor 0, as you specified

    # --- Motor 0 Setup ---
    dir_pin = Pin(D0, Pin.OUT)
    dir_pin.value(1) # Set initial direction (forward)
    
    pwm_pin = Pin(M0, Pin.OUT) # PWM enable pin
    timer4 = Timer(4, freq=20000) # Timer for PWM
    ch1 = timer4.channel(1, Timer.PWM, pulse_width_percent=0, pin=pwm_pin)
    
    print("Motor 0 using PWM pin {} and Direction pin {}".format(M0, D0))

    # --- Encoder Pin Setup for Motor 0 ---
    try:
        encoder_pin = Pin(ENCODER_PIN_NAME, Pin.IN, Pin.PULL_UP)
        # Attach the interrupt handler to the encoder pin on a falling edge
        encoder_pin.irq(trigger=Pin.IRQ_FALLING, handler=_simple_encoder_callback)
#       extint = ExtInt(encoder_pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, _simple_encoder_callback)
        print("Encoder pin {} set up: IN, PULL_UP, IRQ_FALLING.".format(ENCODER_PIN_NAME))

    except Exception as e:
        print("Error setting up encoder pin {}: {}".format(ENCODER_PIN_NAME, e))
        # It's critical that encoder setup works, so exit if it fails
        return 

    # --- Loop Variables for Reporting ---
    last_report_time_ms = utime.ticks_ms()
    report_interval_ms = 500 # Report encoder data every 0.5 seconds

    print("Starting motor sequence. Monitoring encoder data…")

    try:
        while True:
            for duty_percent in range(0, 101, 5): # Go up to 100% duty cycle
                period = timer4.period()  # Get current timer period
                value = int((duty_percent / 100.0) * period) # Calculate pulse width
                ch1.pulse_width(value)
                
                # --- Encoder Data Reporting ---
                if utime.ticks_diff(utime.ticks_ms(), last_report_time_ms) >= report_interval_ms:
                    current_ticks = encoder_state[0]
                    last_valid_interval_us = encoder_state[2] # The interval between last two pulses
                    current_pin_value = encoder_pin.value() # Raw digital value of the pin

                    print("M0: PWM={}%, Ticks={}, Interval={}us, PinValue={}".format(
                        duty_percent, current_ticks, last_valid_interval_us, current_pin_value))
                    last_report_time_ms = utime.ticks_ms()
                
                time.sleep(0.1) # Shorter sleep for more responsive reporting during ramp-up
            
            time.sleep(1) # Pause at max speed (or ramp-up finished)

            for duty_percent in range(100, -1, -5): # Go down to 0% duty cycle
                period = timer4.period()
                value = int((duty_percent / 100.0) * period)
                ch1.pulse_width(value)

                # --- Encoder Data Reporting ---
                if utime.ticks_diff(utime.ticks_ms(), last_report_time_ms) >= report_interval_ms:
                    current_ticks = encoder_state[0]
                    last_valid_interval_us = encoder_state[2]
                    current_pin_value = encoder_pin.value()
                    
                    print("M0: PWM={}%, Ticks={}, Interval={}us, PinValue={}".format(
                        duty_percent, current_ticks, last_valid_interval_us, current_pin_value))
                    last_report_time_ms = utime.ticks_ms()
                
                time.sleep(0.1) # Shorter sleep for responsive reporting during ramp-down

            time.sleep(1) # Pause at 0 speed

    except KeyboardInterrupt:
        print("\nCtrl-C caught. Stopping motor.")
    except Exception as e:
        print("{} raised: {}".format(type(e), e))
    finally:
        print("Cleaning up...")
        if ch1:
            ch1.pulse_width_percent(0) # Set PWM to 0
        if timer4:
            timer4.deinit() # Deinitialize the timer
        if 'encoder_pin' in locals():
            encoder_pin.irq(None) # Detach the interrupt handler from the encoder pin
        print("Stopped and cleaned up.")

if __name__ == '__main__':
    main()

#EOF
