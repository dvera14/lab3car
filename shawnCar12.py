#! Revision: obstacle-avoid with stop + buzzer alert
from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
from buzzer import Buzzer

import time
import math

# --- Hardware initialization ---
PWM = Ordinary_Car()
servo = Servo()
ultrasonic = Ultrasonic()
IF = Infrared()
buzzer = Buzzer()

TURN_TIME = 0.55
TURN_CHECK_DT = 0.02
PROBE_TIME = 0.2

AVOID_STEER_TIME = 0.60   # seconds to steer out (left arc) – softer
AVOID_STRAIGHT_TIME = 0.80 # long run past obstacle
AVOID_MERGE_TIME = 0.45  # steer back (right arc) – stronger/longer
AVOID_FINAL_CORRECT_TIME = 0.45  # extra snap to square up
AVOID_FINAL_STRAIGHT = 1.10
STOP_TIME = 0.4

# ---------- obstacle hysteresis + debounce ----------
STOP_CM = 26      # trigger stop/avoid at or below this (raised for earlier catch)
CLEAR_CM = 20         # must exceed this to resume (hysteresis)
OBSTACLE_HITS_TO_STOP = 3   # consecutive obstacle reads before stopping
CLEAR_HITS_TO_GO = 4        # consecutive clear reads before resuming
BYPASS_TIME = 0.60   # unused now (legacy avoid param)
# ----------------------------------------------------

# ---------- robust ultrasonic read (filters spikes/None) ----------
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
# -----------------------------------------------------------------

def drive_for(lf, lr, rf, rr, duration):
    PWM.set_motor_model(lf, lr, rf, rr)
    time.sleep(duration)


def avoid_obstacle():
    """Single-pass lane change around a front obstacle, then re-merge (shape -\\___/--)."""
    # Steer left ~45 deg to leave lane (pivot bias)
    drive_for(-1600, -1600, 1800, 1800, AVOID_STEER_TIME)
    drive_for(0,0,0,0, STOP_TIME)
    # Long straight to get fully past the obstacle
    drive_for(950, 950, 950, 950, AVOID_STRAIGHT_TIME)
    drive_for(0,0,0,0, STOP_TIME)
    # Steer right to run parallel and begin merge
    drive_for(1900, 1900, -1900, -1900, AVOID_MERGE_TIME)
    drive_for(0,0,0,0, STOP_TIME)
    # Short straight before final correction
    drive_for(900, 900, 900, 900, 0.90)
    drive_for(0,0,0,0, STOP_TIME)
    # Extra snap right to finish aligning back to straight
    drive_for(1600, 1600, -1800, -1800, AVOID_FINAL_CORRECT_TIME)
    drive_for(0,0,0,0, STOP_TIME)
    # Long straight to settle back into the original lane
    drive_for(900, 900, 900, 900, 0.70)
    drive_for(0,0,0,0, STOP_TIME)
    drive_for(-1600, -1600, 1800, 1800, AVOID_FINAL_CORRECT_TIME)
    drive_for(0,0,0,0, STOP_TIME)
    drive_for(900, 900, 900, 900, AVOID_FINAL_STRAIGHT)
    PWM.set_motor_model(0, 0, 0, 0)


obstacle_hits = 0
in_avoidance = False

while True:
    try:
        infrared_value = int(IF.read_all_infrared())

        # Default: drive straight ahead
        PWM.set_motor_model(800, 800, 800, 800)
        # If all sensors see the line (all black), stop and wait for clear path
        if infrared_value == 7:
            PWM.set_motor_model(0, 0, 0, 0)
            clear_hits = 0
            while True:
                distance = distance_cm()
                if (distance is not None) and (distance >= CLEAR_CM):
                    clear_hits += 1
                else:
                    clear_hits = 0
                if clear_hits >= CLEAR_HITS_TO_GO:
                    break
                time.sleep(0.1)

            time.sleep(0.05)
            continue

        # Ultrasonic obstacle check while cruising straight
        distance = distance_cm()

        # Skip retriggering while in avoidance until we've cleared the obstacle
        if in_avoidance:
            if (distance is not None) and (distance >= CLEAR_CM):
                obstacle_hits += 1
            else:
                obstacle_hits = 0
            if obstacle_hits >= CLEAR_HITS_TO_GO:
                in_avoidance = False
                obstacle_hits = 0
            continue

        if (distance is not None) and (distance <= STOP_CM):
            obstacle_hits += 1
        else:
            obstacle_hits = 0

        if obstacle_hits >= OBSTACLE_HITS_TO_STOP:
            PWM.set_motor_model(0, 0, 0, 0)
            buzzer.set_state(True)
            time.sleep(0.5)
            buzzer.set_state(False)
            time.sleep(0.1)
            avoid_obstacle()
            obstacle_hits = 0
            in_avoidance = True  # suppress re-detect until clear distance
            time.sleep(0.05)

    except KeyboardInterrupt:
        PWM.set_motor_model(0, 0, 0, 0)
        print("\nEnd of program")
        break
