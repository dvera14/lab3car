from ultrasonic import Ultrasonic       # not used here, but kept for consistency
from motor import Ordinary_Car
from servo import Servo                 # not used here
from infrared import Infrared
from adc import ADC                     # not used here
import time

class Car:
    def __init__(self):
        self.motor = Ordinary_Car()
        self.infrared = Infrared()
        self.car_record_time = 0.0

    def mode_infrared(self):
        # Limit read/update rate a little bit
        if (time.time() - self.car_record_time) > 0.02:
            self.car_record_time = time.time()
            infrared_value = self.infrared.read_all_infrared()

            # Basic mapping from your example:
            # 2  -> straight
            # 4/6 -> pivot (one side) to correct
            # 1/3 -> pivot (other side) to correct
            # 7  -> stop (intersection/end)
            if infrared_value == 2:
                self.motor.set_motor_model(800, 800, 800, 800)
            elif infrared_value == 4:
                self.motor.set_motor_model(-1500, -1500, 2500, 2500)
            elif infrared_value == 6:
                self.motor.set_motor_model(-2000, -2000, 4000, 4000)
            elif infrared_value == 1:
                self.motor.set_motor_model(2500, 2500, -1500, -1500)
            elif infrared_value == 3:
                self.motor.set_motor_model(4000, 4000, -2000, -2000)
            elif infrared_value == 7:
                self.motor.set_motor_model(0, 0, 0, 0)
            else:
                # If sensors see nothing (e.g., 0), just slow forward:
                self.motor.set_motor_model(1000, 1000, 1000, 1000)

    def close(self):
        try:
            self.motor.set_motor_model(0, 0, 0, 0)
        except:
            pass

def test_car_infrared():
    car = Car()
    try:
        while True:
            car.mode_infrared()
            time.sleep(0.005)
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

if __name__ == '__main__':
    test_car_infrared()
