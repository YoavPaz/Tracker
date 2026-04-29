from robot.tracker import tracker
from robot.camera_controller import camera_controller
from robot.PID import PID
from robot.MPU6050 import MPU6050
from machine import I2C, Pin
import time

camera = camera_controller(tx=2, rx=3)
pid = PID(0 # sp
          ,1.5 # kp
          ,0.5 # ki
          ,0.5 # kd
          )
gyro_i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)
gyro = MPU6050(gyro_i2c)
trac = tracker([9, 8], camera, pid, gyro)

trac.run_servo(0, 90)
trac.run_servo(1, 90)

time.sleep(1)

while True:
    trac.follow_line()

