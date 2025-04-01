import time
import board 
import adafruit_hcsr04
import numpy as np
sonar1 = adafruit_hcsr04.HCSR04(trigger_pin=board.D16, echo_pin=board.D32)
sonar2 = adafruit_hcsr04.HCSR04(trigger_pin=board.D18, echo_pin=board.D36)
sonar3 = adafruit_hcsr04.HCSR04(trigger_pin=board.D22, echo_pin=board.D38)

distance_stack = np.zeros(3, 1)
dt = 1/25
vel_max = 200

while True:
    if min(sonar1.distance, sonar2.distance, sonar3.distance) <= 100:
        pos_des = [0, 1, 0]
        distance_stack_temp = np.array(sonar1.distance, sonar2.distance, sonar3.distance).T
        distance_stack = np.hstack(distance_stack, distance_stack_temp)
        vel_check = 0
        if distance_stack.shape[1] > 5:
            vel_check1 = np.mean(np.abs(distance_stack[:, -4] - distance_stack[:,-5])/dt)
            vel_check2 = np.mean(np.abs(distance_stack[:, -3] - distance_stack[:,-4])/dt)
            vel_check3 = np.mean(np.abs(distance_stack[:, -2] - distance_stack[:,-3])/dt)
            vel_check4 = np.mean(np.abs(distance_stack[:, -1] - distance_stack[:,-2])/dt)
            vel_check = np.mean(np.array([vel_check1, vel_check2, vel_check3, vel_check4]))
        speed_des = np.min(2*vel_check, vel_max)
        speed_des = np.min(speed_des, 0)
        vel_des = speed_des * pos_des


