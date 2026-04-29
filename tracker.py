from machine import Pin, PWM
import time

class tracker:
    def __init__(self, servo_pins, camera_controller, pid, gyro):
        self.servo1 = PWM(Pin(servo_pins[0]))
        self.servo2 = PWM(Pin(servo_pins[1]))
        
        self.gyro = gyro
        self.gyro.reset()
        time.sleep(0.2)
        
        self.servo1.freq(50)
        self.servo2.freq(50)

        self.MIN_US = [400, 500]
        self.STOP_US = [1500, 1500]
        self.MAX_US = [2600, 2500]

        print("Tracker initialized on pins:", servo_pins)

        self.camera = camera_controller
        self.pid = pid

    def us_to_duty(self, microseconds):
        return int((microseconds / 20000) * 65535)

    def angle_to_us(self, angle, index):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        if angle == 90:
            return self.STOP_US[index]

        if angle < 90:
            return self.MIN_US[index] + (angle / 90) * (
                self.STOP_US[index] - self.MIN_US[index]
            )
        else:
            return self.STOP_US[index] + ((angle - 90) / 90) * (
                self.MAX_US[index] - self.STOP_US[index]
            )

    def run_servo(self, index, angle):
        pulse_us = self.angle_to_us(angle, index)
        duty = self.us_to_duty(pulse_us)

        if index == 0:
            self.servo1.duty_u16(duty)
        elif index == 1:
            self.servo2.duty_u16(duty)
        else:
            raise ValueError("Invalid servo index")
    
    def get_gyro_yaw(self):
        self.gyro.update()
        return self.gyro.get_yaw()

    def move_forward(self, speed, dspeed=0):
        # speed range: -90 to 90
        self.run_servo(0, 90 + speed + dspeed + 20)
        self.run_servo(1, 90 - speed + dspeed)

    def read_camera_values(self):
        if self.camera.read_uart():
            print("is waiting")
        return self.camera.current_value
    
    def brake(self):
        self.run_servo(0, 90)
        self.run_servo(1, 90)
    
    def gyro_turn(self, sp, bonus_speed=0):
        yaw = self.get_gyro_yaw()
        while yaw < sp - 0.3 or yaw > sp + 0.3:
            # while not reached sp...
            yaw = self.get_gyro_yaw()
            print(yaw)
            error = self.pid.pid_calc(yaw, sp, 0.8, 0.4, 0.2)
            self.run_servo(0, 90 + error + bonus_speed)
            self.run_servo(1, 90 + error + bonus_speed)
        self.brake()

    def turn_and_check(self, direction, speed=20):
        print("Green detected -> move forward first")
        
        self.move_forward(30)
        time.sleep(0.5)
        self.brake()
        
        self.gyro_turn(self.get_gyro_yaw() - 25 if direction == 1 else self.get_gyro_yaw() + 25)
        print("Starting continuous scan rotation")

        start_time = time.ticks_ms()

        while True:
            self.gyro.update()
            camera_values = self.read_camera_values()

            if camera_values != 999:
                print("Line found!")
                break

            if time.ticks_diff(time.ticks_ms(), start_time) > 5000:
                print("Timeout reached")
                break

            if direction == 1:
                self.run_servo(0, 90 + speed)
                self.run_servo(1, 90 + speed)
            else:
                self.run_servo(0, 90 - speed)
                self.run_servo(1, 90 - speed)

            time.sleep(0.01)

        self.brake()
        self.camera.wait = False

        # Force flush stale green value
        self.camera.current_value = 999

        print("Scan finished — waiting for non-green reading")

        # Wait until OpenMV stops sending green codes
        timeout = time.ticks_ms()
        while True:
            val = self.read_camera_values()
            if val not in (1001, 1002):
                break
            if time.ticks_diff(time.ticks_ms(), timeout) > 3000:
                print("Green flush timeout")
                self.camera.current_value = 999
                break
            time.sleep(0.05)
        
    def follow_line(self):
        camera_values = self.read_camera_values()

        if camera_values == 999:
            # line lost
            return

        elif camera_values == 1001:
            # RIGHT GREEN
            self.turn_and_check(1)
            self.move_forward(20)
            time.sleep(0.5)
            self.brake()
            print("finished green")
            return

        elif camera_values == 1002:
            # LEFT GREEN
            self.turn_and_check(-1)
            self.move_forward(20)
            time.sleep(0.5)
            self.brake()
            print("finished green")
            return

        elif camera_values == 1003:
            print("moving 360!")
            return

        elif camera_values == 1100:
            print("Moving forward!")
            self.move_forward(90)
            time.sleep(1)
            self.move_forward(0)
            self.camera.wait = False
            return

        self.run_based_pid(camera_values)

    def run_based_pid(self, e):
        error = self.pid.pid_calc(e)

        error = max(-60, min(60, error))

        base = 20

        left = base + error
        right = base - error

        self.run_servo(0, 90 + left + 30)
        self.run_servo(1, 90 - right)
