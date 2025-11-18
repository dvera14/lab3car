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

TURN_TIME = 0.55
TURN_CHECK_DT = 0.02
PROBE_TIME = 0.2

# ---------- added: obstacle hysteresis + bypass time ----------
STOP_CM = 35         # trigger stop/avoid at or below this
CLEAR_CM = 40        # must exceed this to resume (hysteresis)
BYPASS_TIME = 0.60   # forward time alongside obstacle during avoid
# --------------------------------------------------------------

def indicate_line_position(sensor_value: int):
    """Show LED color based on which infrared sensors currently see the line."""
    color_map = {
        0: (255, 0, 0),    # no line detected
        1: (255, 165, 0),  # line under left sensor(s)
        2: (0, 255, 0),    # centered on the line
        3: (255, 215, 0),  # drifting left (left + center)
        4: (30, 144, 255), # line under right sensor(s)
        6: (65, 105, 225), # drifting right (center + right)
        7: (255, 255, 255) # all sensors on (intersection/crossing)
    }
    r, g, b = color_map.get(sensor_value, (255, 0, 255))
    led.ledIndex(0x04, r, g, b)

def take_right_turn():
    """Pivot right at a crossroad until the center sensor finds the new line."""
    turn_start = time.time()
    PWM.set_motor_model(4000, 4000, -2000, -2000)
    while time.time() - turn_start < TURN_TIME:
        turn_value = int(IF.read_all_infrared())
        indicate_line_position(turn_value)
        if turn_value & 0b010:
            break
        time.sleep(TURN_CHECK_DT)
    PWM.set_motor_model(2000, 2000, 2000, 2000)
    time.sleep(PROBE_TIME)
    PWM.set_motor_model(0, 0, 0, 0)

# ---------- added: robust ultrasonic read (works if spikes/None) ----------
def distance_cm():
    vals = []
    for _ in range(3):
        try:
            d = ultrasonic.get_distance()
            if d and d > 0:
                vals.append(d)
        except:
            pass
        time.sleep(0.01)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals)//2]
# -------------------------------------------------------------------------

while True:
    try:
        # ===================== DRIVING / LINE-FOLLOWING =====================
        infrared_value = int(IF.read_all_infrared())
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
            take_right_turn()
            continue
        # ---------- added: search-on-loss so it doesn't stall ----------
        elif infrared_value == 0:
            # gentle left spin to re-acquire line
            PWM.set_motor_model(-1500, -1500, 2500, 2500)
        # ----------------------------------------------------------------

        indicate_line_position(infrared_value)

        # Ultrasonic obstacle check (stop-and-wait; no avoidance drive)
        distance = distance_cm()
        if (distance is not None) and (distance <= STOP_CM):
            PWM.set_motor_model(0, 0, 0, 0)

            buzzer.set_state(True)
            time.sleep(1)
            buzzer.set_state(False)

            led.colorBlink(0)

            # hold here until obstacle is cleared (hysteresis to restart)
            while True:
                distance = distance_cm()
                if (distance is not None) and (distance >= CLEAR_CM):
                    break
                time.sleep(0.1)

    except KeyboardInterrupt:
        PWM.set_motor_model(0, 0, 0, 0)
        print("\nEnd of program")
        break
