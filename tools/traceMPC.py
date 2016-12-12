import matplotlib.pyplot as pl
import csv


tauList = []
tauDotList = []
qList = []
qDotList = []
uList = []

finaltauList = []
finaltauDotList = []
finalqList = []
finalqDotList = []
finaluList = []

tauListList = []
tauDotListList = []
qListList = []
qDotListList = []
uListList = []

''' position '''
path = '../_build/cpp/resultsMPC.csv'
T=5;
N = 30;
		
			
with open(path,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    j = -1
    for row in reader:
		if i ==2:
			if(j<T):
				tauList.append(float(row[0]))
				tauDotList.append(float(row[1]))
				qList.append(float(row[2]))
				qDotList.append(float(row[3]))
				uList.append((float(row[4])))
				j+=1
			else:
				tauListList.append(tauList)
				tauDotListList.append(tauDotList)
				qListList.append(qList)
				qDotListList.append(qDotList)
				uListList.append(uList)
				tauList = [float(row[0])]
				tauDotList = [float(row[1])]
				qList = [float(row[2])]
				qDotList = [float(row[3])]
				uList = [float(row[4])]
				j = 0
		if i==1:
			i = 2 			
		if i==0:
			T = int(row[0])
			N = int(row[1]) - 1
			i = 1			
 
for i in range(N):
	finaltauList.append(tauListList[i][0])
	finaltauDotList.append(tauDotListList[i][0])
	finalqList.append(qListList[i][0])
	finalqDotList.append(qDotListList[i][0])
	finaluList.append(uListList[i][0])           

#~ print len(tauListList[0])
#~ print len(tauListList[1])
#~ print len(tauListList[2])
#~ print len(tauListList[3])  
#~ print tauListList[0]	
#~ print tauListList[1]
#~ print tauListList[2]	
#~ print tauListList[3]

fig1 = pl.figure()
fig2 = pl.figure()

ax1 = fig1.add_subplot(221)
bx1 = fig1.add_subplot(222)
cx1 = fig1.add_subplot(223)
dx1 = fig1.add_subplot(224)
ax2 = fig2.add_subplot(111)

for i in range(0,N,20):
	ax1.plot(range(i,i+T+1,1),tauListList[i])
	bx1.plot(range(i,i+T+1,1),tauDotListList[i])
	cx1.plot(range(i,i+T+1,1),qListList[i])
	dx1.plot(range(i,i+T+1,1),qDotListList[i])
	ax2.plot(range(i,i+T+1,1),uListList[i])
	
ax1.plot(finaltauList,'g+')
bx1.plot(finaltauDotList,'g+')
cx1.plot(finalqList,'g+')
dx1.plot(finalqDotList,'g+')
ax2.plot(finaluList,'g+')

ax1.grid()
bx1.grid()
cx1.grid()
dx1.grid()
ax2.grid()
pl.show()

