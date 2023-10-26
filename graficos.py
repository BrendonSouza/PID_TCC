import numpy as np
import matplotlib.pyplot as plt
import json

#le os dados dos arquivos de velocidades_direita.txt, atribui a uma variável as velocidades e a outra os tempos
with open('velocidades_direita.txt', 'r') as f:
    linesDireita = f.readlines()

velocidades_direita = []
tempos_direita = []

for line in linesDireita:
    data = json.loads(line)
    velocidades_direita.append(data['rpm'])
    tempos_direita.append(data['time'])

# converte todos os dados de velocidade para rpm
velocidades_direita = np.array(velocidades_direita)/6
velocidades_direita = np.array(velocidades_direita)/60

# plota o gráfico de velocidade em função do tempo

plt.plot(tempos_direita, velocidades_direita)
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (rps)')
plt.title('Velocidade da roda direita')

plt.show()

#corta os dados até o tempo de 1 segundo

