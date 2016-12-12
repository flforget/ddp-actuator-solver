from matplotlib.pyplot import *
import csv


tau1List = []
tauDot1List = []
q1List = []
qDot1List = []
u1List = []
tau2List = []
tauDot2List = []
q2List = []
qDot2List = []
u2List = []

tau2ListList = []
tauDot2ListList = []
q2ListList = []
qDot2ListList = []
u2ListList = []

''' position '''
path1 = '../_build/cpp/results1.csv'
path2 = '../_build/cpp/results2.csv'

with open(path1,'r') as dataFile1:
    reader = csv.reader(dataFile1)
    i = 0
    j = 0
    for row in reader:
        if i ==1:
            tau1List.append(float(row[0]))
            tauDot1List.append(float(row[1]))
            q1List.append(float(row[2]))
            qDot1List.append(float(row[3]))
            u1List.append((float(row[4])))
        if i==0:
            i = 1

with open(path2,'r') as dataFile2:
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
            i = 1

fig1 = figure()

subplot(221)
hold(1)
plot(tau1List)
for i in range(N):
    plot(tau2ListList[i])
title('joint position',fontsize=32)
grid()

subplot(222)
hold(1)
plot(tauDot1List)
for i in range(N):
    plot(tauDot2ListList[i])
title('joint speed',fontsize=32)
grid()

subplot(223)
hold(1)
plot(q1List)
for i in range(N):
    plot(q2ListList[i])
title('motor position',fontsize=32)
grid()

subplot(224)
hold(1)
plot(u1List)
for i in range(N):
    plot(u2ListList[i])
title('motor current',fontsize=32)
grid()


figure()
hold(1)
plot(tau1List)
for i in range(N):
    plot(tau2ListList[i])
title('joint position',fontsize=32)
grid()


show()

