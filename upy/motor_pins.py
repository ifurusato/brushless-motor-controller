#!/micropython

'''
  * Drives 4 motors
  * Uses hardware PWM output for speed control
  * Uses GPIO output for direction control
  * Sets up input capture for reading PWM input (e.g., from encoders or feedback signals)

Motor Pin Mapping on STM32F405 (WeAct Board) with X/Y Pin Names

    +---------+------------+-----------+------------+
    | Motor   | PWM Out    | Direction | PWM In     |
    +---------+------------+-----------+------------+
    | Motor 1 | PB6 (X9)   | PB5 (X11) | PB7 (X10)  |
    |         | TIM4_CH1   | GPIO      | TIM4_CH2   |
    +---------+------------+-----------+------------+
    | Motor 2 | PB8 (Y3)   | PC12 (Y11)| PB9 (Y4)   |
    |         | TIM4_CH3   | GPIO      | TIM4_CH4   |
    +---------+------------+-----------+------------+
    | Motor 3 | PB12 (Y5)  | PB13 (Y6) | PB14 (Y7)  |
    |         | TIM1_CH1   | GPIO      | TIM1_CH2   |
    +---------+------------+-----------+------------+
    | Motor 4 | PC10 (Y12) | PC11 (Y10)| PB3 (X17)  |
    |         | TIM8_CH3   | GPIO      | TIM2_CH2   |
    +---------+------------+-----------+------------+

Note:
- PWM Out   = PWM output pin with Timer channel
- Direction = Direction pin (GPIO output)
- PWM In    = PWM encoder input pin with Timer channel
- Pins chosen to prioritize physical proximity
'''

from pyb import Pin, Timer

# Initialize timers
tim4 = Timer(4, freq=1000)   # Motor 1 & 2 PWM Out + PWM In
tim1 = Timer(1, freq=1000)   # Motor 3 PWM Out + PWM In
tim8 = Timer(8, freq=1000)   # Motor 4 PWM Out
tim2 = Timer(2, freq=1000)   # Motor 4 PWM In

# --- Motor 1 ---
m1_pwm = tim4.channel(1, Timer.PWM, pin=Pin('X9'))   # PB6 TIM4_CH1 PWM Out
m1_dir = Pin('X11', Pin.OUT)                          # PB5 GPIO Dir
m1_dir.value(1)
m1_in  = tim4.channel(2, Timer.IC, pin=Pin('X10'), polarity=Timer.BOTH)  # PB7 TIM4_CH2 PWM In

# --- Motor 2 ---
m2_pwm = tim4.channel(3, Timer.PWM, pin=Pin('Y3'))   # PB8 TIM4_CH3 PWM Out
m2_dir = Pin('Y11', Pin.OUT)                         # PC12 GPIO Dir
m2_dir.value(1)
m2_in  = tim4.channel(4, Timer.IC, pin=Pin('Y4'), polarity=Timer.BOTH)  # PB9 TIM4_CH4 PWM In

# --- Motor 3 ---
m3_pwm = tim1.channel(1, Timer.PWM, pin=Pin('Y5'))   # PB12 TIM1_CH1 PWM Out
m3_dir = Pin('Y6', Pin.OUT)                          # PB13 GPIO Dir
m3_dir.value(1)
m3_in  = tim1.channel(2, Timer.IC, pin=Pin('Y7'), polarity=Timer.BOTH)  # PB14 TIM1_CH2 PWM In

# --- Motor 4 ---
m4_pwm = tim8.channel(3, Timer.PWM, pin=Pin('Y12'))  # PC10 TIM8_CH3 PWM Out
m4_dir = Pin('Y10', Pin.OUT)                         # PC11 GPIO Dir
m4_dir.value(1)
m4_in  = tim2.channel(2, Timer.IC, pin=Pin('X17'), polarity=Timer.BOTH) # PB3 TIM2_CH2 PWM In

print("Four motors initialized with updated pin configuration for WeAct STM32F405.")

