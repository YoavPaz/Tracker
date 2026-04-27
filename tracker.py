from machine import Pin, PWM
import time

class tracker:
    def __init__(self, servo_pins, camera_controller, pid):
        self.servo1 = PWM(Pin(servo_pins[0]))
        self.servo2 = PWM(Pin(servo_pins[1]))

        self.servo1.freq(50)
        self.servo2.freq(50)

        self.MIN_US = [400, 500]
        self.STOP_US = [1500, 1500]
        self.MAX_US = [2600, 2500]

        print("Tracker initialized on pins:", servo_pins)
        
        self.camera = camera_controller
        self.pid = pid

    def us_to_duty(self, microseconds):
        # 50Hz = 20ms period
        # duty_u16 range: 0 - 65535
        return int((microseconds / 20000) * 65535)

    def angle_to_us(self, angle, index):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        if angle == 90:
            return self.STOP_US[index]

        if angle < 90:
            return self.MIN_US[index] + (angle / 90) * (self.STOP_US[index] - self.MIN_US[index])
        else:
            return self.STOP_US[index] + ((angle - 90) / 90) * (self.MAX_US[index] - self.STOP_US[index])

    def run_servo(self, index, angle):
        pulse_us = self.angle_to_us(angle, index)
        duty = self.us_to_duty(pulse_us)

        if index == 0:
            self.servo1.duty_u16(duty)
        elif index == 1:
            self.servo2.duty_u16(duty)
        else:
            raise ValueError("Invalid servo index")

        #print(f"Servo {index} → angle {angle} → {pulse_us:.1f}us → duty {duty}")
    
    def move_forward(self, speed, dspeed = 0): # speed range -90 to 90
        self.run_servo(0, 90 + speed + dspeed + 20) # the final + is a bonus for the servo 0 because it is weaker then servo 1!
        self.run_servo(1, 90 - speed + dspeed)
    
    def read_camera_values(self):
        if (self.camera.read_uart()):
            print("is waiting")
        return self.camera.current_value
    
    def follow_line(self):
        camera_values = self.read_camera_values()

        if camera_values == 999:
            # line lost → stop or search
            #self.move_forward(-90)
            return
        
        if camera_values == 1001:
            #print("moving right")
            self.move_forward(70)
            time.sleep(0.5)
            self.run_based_pid(90)
            time.sleep(0.6)
            self.camera.wait = False
            self.move_forward(0)
            time.sleep(1)
            return
        elif camera_values == 1002:
            #print("moving left")
            self.move_forward(70)
            time.sleep(0.3)
            self.run_based_pid(-90)
            time.sleep(0.6)
            self.camera.wait = False
            self.move_forward(0)
            time.sleep(1)
            return
        elif camera_values == 1003:
            print("moving 360!")
            return
        elif camera_values == 1100: # move forward
            print("Moving forward!")
            self.move_forward(90)
            time.sleep(1)
            self.move_forward(0)
            self.camera.wait = False
            
        self.run_based_pid(camera_values)
        
        
    def run_based_pid(self, e):
        error = self.pid.pid_calc(e)

        error = max(-60, min(60, error))

        #print(f"[PID] SP , PV {camera_values}, error {error}")

        base = 20

        left = base + error
        right = base - error

        self.run_servo(0, 90 + left + 30) # 20 to help the servo (weak servo)
        self.run_servo(1, 90 - right)
