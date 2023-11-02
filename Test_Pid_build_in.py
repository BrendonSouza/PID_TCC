#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick  # Importa a classe EV3Brick do módulo pybricks.hubs
from pybricks.ev3devices import Motor  # Importa a classe Motor do módulo pybricks.ev3devices
from pybricks.parameters import Port  # Importa a classe Port do módulo pybricks.parameters
from pybricks.robotics import DriveBase  # Importa a classe DriveBase do módulo pybricks.robotics
import math  # Importa o módulo math para operações matemáticas
import time
import json

ev3 = EV3Brick()  # Cria uma instância da classe EV3Brick para interagir com o bloco EV3

left_motor = Motor(Port.A)  # Cria uma instância da classe Motor para o motor esquerdo
right_motor = Motor(Port.D)  # Cria uma instância da classe Motor para o motor direito

# Configure as medidas da roda
wheel_diameter = 43.2  # Diâmetro da roda em milímetros
entre_eixos = 143 # Distância entre as rodas em milímetros

# Cria uma instância da classe DriveBase para controlar o movimento do robô
robot = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter, axle_track=entre_eixos)

raio = wheel_diameter/2
start = time.time()
velocidades_direita = [{"rpm": 0, "time": 0}]
velocidades_esquerda = [{"rpm": 0, "time": 0}]
targetSpeed = 600
# desactivate pid actuation
left_motor.control.pid(400, 1200, 5, 23,5,0)
# right_motor.control.pid(1200, 0, 0, 0,0,0)
# (400, 1200, 5, 23, 5, 0)
while True:
    # right_motor.run(targetSpeed)
    left_motor.run(targetSpeed)

    print(left_motor.control.pid())
    # print(right_motor.control.pid())
#   salva num array o RPM de cada roda e o tempo de medição
    # velocidades_direita.append({'rpm': right_motor.speed(), 'time': time.time() - start})
    velocidades_esquerda.append({'rpm': left_motor.speed(), 'time': time.time() - start})
    # print(velocidades_direita[-1]['rpm'], velocidades_esquerda[-1]['rpm'])
    if time.time() - start > 3:
        break
robot.stop()

# salva num arquivo os dados de RPM e tempo de medição

# with open('velocidades_direita_PID_INTERNO.txt', 'w') as f:
#     for item in velocidades_direita:
#         # escreva o item entre aspas duplas
#         f.write(json.dumps(item) + '\n')

#         # f.write("%s\n" % item)
 

with open('velocidades_esquerda_PID_INTERNO.txt', 'w') as f:
    for item in velocidades_esquerda:
        f.write(json.dumps(item) + '\n')
        # f.write("%s\n" % item)



