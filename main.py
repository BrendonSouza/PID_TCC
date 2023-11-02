#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick  # Importa a classe EV3Brick do módulo pybricks.hubs
from pybricks.ev3devices import Motor  # Importa a classe Motor do módulo pybricks.ev3devices
from pybricks.parameters import Port  # Importa a classe Port do módulo pybricks.parameters
from pybricks.robotics import DriveBase  # Importa a classe DriveBase do módulo pybricks.robotics
import math  # Importa o módulo math para operações matemáticas
import time
import _thread
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

Nd = 0 #pulsos roda direita
Ne = 0 #pulsos roda esquerda
Pd = 0 #total de pulsos roda direita
Pe = 0 #total de pulsos roda esquerda.
X = []
Y= []
teta = []
tempos = []

Kp = 3  # Ganho proporcional
Ki = 0.16  # Ganho integral
Kd = 4.96  # Ganho derivativo

# Variáveis para armazenar valores anteriores e somas do erro para o cálculo integral
integral = 0
ultimo_erro = 0
setpoint = 0 

dados_Odo = [{'X': 0, 'Y': 0, 'teta': 0, 'tempo': 0}]
setpoint = 0



def pid_control(erro, delta_time):
  global integral, ultimo_erro

  # Componente proporcional
  proporcional = Kp * erro

  if erro == 0:
    integral = 0
  else:
    integral = integral *erro


  integrativo = Ki * integral

  # Componente derivativo
  derivativo = Kd * (erro - ultimo_erro)

  # Atualiza o erro anterior para a próxima iteração
  ultimo_erro = erro

  # Saída do PID
  output = (proporcional + integrativo + derivativo) 
  return output


def thread_calculo():
  global Pe, Pd, Ne, Nd
  start = time.time()
  while robot.distance() < 1000:
    
    Pe = Pe + Ne
    Pd = Pd + Nd 
    Ne = left_motor.angle() - Pe 
    Nd = right_motor.angle() - Pd 
    teta.append(teta[-1] + (2*math.pi * ((Ne * raio) - (Nd * raio))/(360*entre_eixos))) 
    X.append(X[-1] + (math.pi * ((Ne * raio) + (Nd * raio))/360  * math.cos(teta[-1]))) 
    Y.append(Y[-1] + (math.pi * ((Ne * raio) + (Nd * raio))/360  * math.sin(teta[-1]))) 
    dados_Odo.append({'X': X[-1], 'Y': Y[-1], 'teta': teta[-1], 'tempo': time.time() - start})
    ev3.screen.print("X: ",X[-1])
    ev3.screen.print("Y: ",Y[-1])
    last_teta = "{:.3f}".format(teta[-1])
    ev3.screen.print("teta: ",last_teta)

teta.append(0)
X.append(0)
Y.append(0)
def move(alvo):
  cont = 0
  ultimo_tempo = time.time()
  global Pe, Pd, Ne, Nd
  _thread.start_new_thread(thread_calculo, ())
  while robot.distance() < alvo:
    erro = setpoint - teta[-1]
    time_now = time.time()
    delta_time = time_now - ultimo_tempo
    ultimo_tempo = time_now
    ajuste = pid_control(erro, delta_time)
    ev3.screen.print("X: ",X[-1])
    ev3.screen.print("Y: ",Y[-1])
    robot.drive(150, ajuste)

def L():
  
  ev3.speaker.beep() 
  move(1000)
  robot.stop()



L()
with open('dadosOdo.txt', 'w') as f:
    for item in dados_Odo:
        f.write(json.dumps(item) + '\n')

ev3.speaker.beep()  # Emite um som para indicar o final do movimento para frente
robot.stop()
