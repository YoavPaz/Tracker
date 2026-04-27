from robot.tracker import tracker
from robot.camera_controller import camera_controller
from robot.PID import PID
import time
#import reset_all_pins

camera = camera_controller(tx=2, rx=3)
pid = PID(0 # sp
          ,0.5 # kp
          ,0 # ki
          ,0 # kd
          )
trac = tracker([9, 8], camera, pid)

trac.run_servo(0, 90)
trac.run_servo(1, 90)

time.sleep(1)

while True:
    #trac.run_servo(0, 60)
    #time.sleep(1)
    
    #trac.run_servo(0, 170)
    #time.sleep(1)
    trac.follow_line()
