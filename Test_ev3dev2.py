#!/usr/bin/env python3
from ev3dev.ev3 import *
import time
import json

# Connect motors
motor_left = Motor('outA')
motor_right = Motor('outD')

# Define PID constants
Kp = 10
Ki = 0
Kd = 0

# Initialize PID variables
integral_left = 0
integral_right = 0
last_error_left = 0
last_error_right = 0

# Define the setpoint (desired speed)
setpoint = 400

velocidades_esquerda = [{'rpm': 0, 'time': 0}]
velocidades_direita = [{'rpm': 0, 'time': 0}]
start = time.time()
speed_left = 0
speed_right = 0
motor_left.speed_p=0 
motor_left.speed_i = 0 
motor_left.speed_d = 0

motor_right.speed_p=0
motor_right.speed_i = 0
motor_right.speed_d = 0
print(motor_left.max_speed, motor_right.max_speed)
    
while True:
    # Calculate error
    error_left = setpoint - motor_left.speed
    error_right = setpoint - motor_right.speed
    
    # Integrate error
    integral_left += error_left
    integral_right += error_right
    
    # Compute derivative
    derivative_left = error_left - last_error_left
    derivative_right = error_right - last_error_right
    
    # Compute PID output
    output_left = Kp*error_left + Ki*integral_left + Kd*derivative_left
    output_right = Kp*error_right + Ki*integral_right + Kd*derivative_right

    speed_left = setpoint + output_left
    speed_right = setpoint + output_right
    if(setpoint + output_left > 1050):
        speed_left = 1050
    if(setpoint + output_right > 1050):
        speed_right = 1050
    if(setpoint + output_left < -1050):
        speed_left = -1050

    if(setpoint + output_right < -1050):
        speed_right = -1050
    # Adjust motor speed
    motor_left.run_forever(speed_sp=speed_left)
    motor_right.run_forever(speed_sp=speed_right)
    
    velocidades_direita.append({'rpm': motor_right.speed, 'time': time.time() - start})
    velocidades_esquerda.append({'rpm': motor_left.speed, 'time': time.time() - start})

    print(motor_left.speed_p, motor_left.speed_i, motor_left.speed_d)

    # Update last error for next loop
    last_error_left = error_left
    last_error_right = error_right

    

    if time.time() - start > 3:
        motor_left.stop(stop_action="brake")
        motor_right.stop(stop_action="brake")
        break

with open('velocidades_direita_ev3dev.txt', 'w') as f:
    for item in velocidades_direita:
        f.write(json.dumps(item) + '\n')

with open('velocidades_esquerda_ev3dev.txt', 'w') as f:
    for item in velocidades_esquerda:
        f.write(json.dumps(item) + '\n')
