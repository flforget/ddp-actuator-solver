from matplotlib.pyplot import *
import numpy as np
import csv

''' position '''
path1 = '../_build/examples/temperature_control/results1.csv'
path2 = '../_build/examples/temperature_control/results2.csv'

names = []
data = []

with open(path1,'r') as dataFile1:
    reader = csv.reader(dataFile1)
    i = 0
    for row in reader:
        if i ==2:
            for j in range(len(names)):
                data[j].append(float(row[j]))
        if i==1:
            T = int(row[0])
            S_NB = int(row[1])
            C_NB = int(row[2])
            for j in range(len(names)):
                data.append([])
            i = 2
        if i==0:
            for name in row:
                names.append(name)
            i = 1
'''with open(path2,'r') as dataFile2:
    reader = csv.reader(dataFile2)
    i = 0
    j = -1
    for row in reader:
        if i ==2:
            if (j < T):
                tau2List.append(float(row[0]))
                tauDot2List.append(float(row[1]))
                q2List.append(float(row[2]))
                qDot2List.append(float(row[3]))
                u2List.append((float(row[4])))
                j += 1
            else:
                tau2ListList.append(tau2List)
                tauDot2ListList.append(tauDot2List)
                q2ListList.append(q2List)
                qDot2ListList.append(qDot2List)
                u2ListList.append(u2List)
                tau2List = [float(row[0])]
                tauDot2List = [float(row[1])]
                q2List = [float(row[2])]
                qDot2List = [float(row[3])]
                u2List = [float(row[4])]
                j = 0
        if i ==1:
            T = int(row[0])
            N = int(row[1]) - 1
            i = 2
        if i==0:
            i = 1'''

fig1 = figure()
for i in range(S_NB):
    subplot(int(S_NB/2)+1,2,int(i+1))
    plot(data[i])
    title(names[i])
    grid()


fig2 = figure()
for i in range(C_NB):
    subplot(int(C_NB/2)+1,2,int(i+1))
    plot(data[i+S_NB])
    title(names[i+S_NB])
    grid()


show()
