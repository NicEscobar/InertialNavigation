import numpy as np

sp = np.array([5,5])
ant = np.array([0,0])
atual = np.array([2,4])

VetorReferencia =  sp - ant
VetorDescolamento = atual - ant

ModuloReferencia = np.sqrt(np.sum(np.square(VetorReferencia)))
ModuloDeslocamento = np.sqrt(np.sum(np.square(VetorDescolamento)))

alfa = np.arccos(VetorReferencia[0]/ModuloReferencia)
theta = np.arccos(VetorDescolamento[0]/ModuloDeslocamento)

print('alfa:', alfa)
print('theta:', theta)
