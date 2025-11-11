from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
from led import Led
from buzzer import Buzzer


import time
import math

# --- Hardware initialization ---
PWM = Ordinary_Car()
led = Led()
buzzer = Buzzer()
servo = Servo()
ultrasonic = Ultrasonic()
IF = Infrared()

# --- State variables ---
driving_state = 1
stopping_state = 0
scanning_state = 0

while True:
    try:
        # ===================== DRIVING / LINE-FOLLOWING =====================
        if driving_state == 1:
            # Infrared line-following logic (unchanged)
            infrared_value = IF.read_all_infrared()
            # print("infrared_value: " + str(infrared_value))

            if infrared_value == 2:
                PWM.set_motor_model(2000, -800, 2000, -800)
            elif infrared_value == 4:
                PWM.set_motor_model(-1500, -1500, 2500, 2500)
            elif infrared_value == 6:
                PWM.set_motor_model(-2000, -2000, 4000, 4000)
            elif infrared_value == 1:
                PWM.set_motor_model(2500, 2500, -1500, -1500)
            elif infrared_value == 3:
                PWM.set_motor_model(4000, 4000, -2000, -2000)
            elif infrared_value == 7:
                PWM.set_motor_model(0, 0, 0, 0)

            # LED rainbow while driving (from your obstacle code)
            led.ledIndex(0x04, 255, 255, 0)     # Red
            led.ledIndex(0x04, 255, 255, 0)   # Orange
            led.ledIndex(0x04, 255, 255, 0)   # Yellow
            led.ledIndex(0x04, 255, 255, 0)     # Green
            led.ledIndex(0x04, 255, 255, 0)   # Cyan-blue
            led.ledIndex(0x04, 255, 255, 0)     # Blue
            led.ledIndex(0x04, 255, 255, 0)   # Purple
            led.ledIndex(0x04, 255, 255, 0) # White

            # Ultrasonic obstacle check (from your obstacle code)
            distance = ultrasonic.get_distance()

            if distance <= 35:
                PWM.set_motor_model(0, 0, 0, 0)
                driving_state = 0
                stopping_state = 1
                scanning_state = 0

                buzzer.set_state(True)
                time.sleep(1)
                buzzer.set_state(False)

                led.colorBlink(0)

        # ===================== STOPPING STATE =====================
        if stopping_state == 1:
            PWM.set_motor_model(0, 0, 0, 0)
            time.sleep(2)

            driving_state = 0
            stopping_state = 0
            scanning_state = 1

        # ===================== SCANNING / AVOIDANCE =====================
        if scanning_state == 1:
            distance = ultrasonic.get_distance()

            if distance <= 35:
                PWM.set_motor_model(1500, 1500, -1500, -1500)
                time.sleep(1)
                PWM.set_motor_model(0, 0, 0, 0)

            if distance >= 35:
                scanning_state = 0
                driving_state = 1
                stopping_state = 0

    except KeyboardInterrupt:
        PWM.set_motor_model(0, 0, 0, 0)
        print("\nEnd of program")
        break