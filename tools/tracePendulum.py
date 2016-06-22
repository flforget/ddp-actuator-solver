import matplotlib.pyplot as pl
import csv


thetaList = []
thetaDotList = []
uList = []

''' position '''
path = '../_build/cpp/resultsPendulum.csv'

with open(path,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    for row in reader:
        if i ==1:
            thetaList.append(float(row[0]))
            thetaDotList.append(float(row[1]))
            uList.append((float(row[2])))
        if i==0:
            i = 1

fig1 = pl.figure()

ax1 = fig1.add_subplot(221)
ax1.plot(thetaList)
ax1.grid()

bx1 = fig1.add_subplot(222)
bx1.plot(thetaDotList)
bx1.grid()

cx1 = fig1.add_subplot(223)
cx1.plot(uList)
cx1.grid()


pl.show()

