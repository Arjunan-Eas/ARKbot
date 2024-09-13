import Jetson.GPIO as GPIO
import time

# Pin Definitions
claw_motor_speed = 33
arm_motor_speed = 32
claw_motor_1 = 37
claw_motor_2 = 38
arm_motor_1 = 36
arm_motor_2 = 40


# GPIO setup
GPIO.setmode(GPIO.BOARD)  # board numbering scheme
GPIO.setup(claw_motor_speed, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(arm_motor_speed, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(claw_motor_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(claw_motor_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(arm_motor_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(arm_motor_2, GPIO.OUT, initial=GPIO.LOW)

# # Create PWM instance on pwm_pin 100Hz frequency
# pwm = GPIO.PWM(claw_motor_speed, 490)  # 100 Hz
# pwm.start(50)  # Start PWM with 0% duty cycle (off)

# pwm = GPIO.PWM(arm_motor_speed, 490)  # 100 Hz
# pwm.start(50)  # Start PWM with 0% duty cycle (off)



try:
    for i in range(1):
        # On/Off control for gpio_pin
        print("claw 1 high")
        GPIO.output(claw_motor_1, GPIO.HIGH)  # Turn on
        GPIO.output(claw_motor_2, GPIO.LOW)
        time.sleep(0.125)  # Wait 1 second
        GPIO.output(claw_motor_1, GPIO.LOW)   # Turn off
        print("off")
        time.sleep(1)  # Wait 1 second

        print("claw 2 high")
        # On/Off control for gpio_pin
        GPIO.output(claw_motor_2, GPIO.HIGH)  # Turn on
        GPIO.output(claw_motor_1, GPIO.LOW)
        time.sleep(0.125)  # Wait 1 second
        GPIO.output(claw_motor_2, GPIO.LOW)   # Turn off
        print("off")
        time.sleep(1)  # Wait 1 second

        print("arm 1 high")
        GPIO.output(arm_motor_1, GPIO.HIGH)  # Turn on
        GPIO.output(arm_motor_2, GPIO.LOW)
        time.sleep(0.125)  # Wait 1 second
        GPIO.output(arm_motor_1, GPIO.LOW)   # Turn off
        print("off")
        time.sleep(1)  # Wait 1 second

        print("arm 2 high")
        GPIO.output(arm_motor_2, GPIO.HIGH)  # Turn on
        GPIO.output(arm_motor_1, GPIO.LOW)
        time.sleep(0.125)  # Wait 1 second
        GPIO.output(arm_motor_2, GPIO.LOW)   # Turn off
        print("off")
        time.sleep(1)  # Wait 1 second

       
except KeyboardInterrupt:
    pass  # Exit gracefully on CTRL+C

finally:
    # pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Clean up GPIO resources
