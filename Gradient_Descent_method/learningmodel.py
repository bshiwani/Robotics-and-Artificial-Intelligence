import csv
import random
import numpy as np
import matplotlib.pyplot as pyplot
import matplotlib.pyplot as plt
import pandas as pd
import pylab as pl

total=[]
avg=[]
arr=[]
final=[]
mdata = open('test123.csv')
data = list(csv.reader(mdata))
col=len(data[0])
row=len(data)
add=[]

for i in range(col):
    arr=[]
    for j in range(row):
        arr.append(float(data[j][i]))
    final.append(list(arr))

for m in range(col):
    add.append(max(final[m]))

for n in range(col):
    for m in range(row):
        final[n][m]=final[n][m]/add[n] # format is column, row
time=[]

#for i in range(row):
 #   time.append(i)
#print(time)
#ts = pd.Series(final[4], index=time)

#ts.plot()
#plt.show()
J=[]
step_size=0.1
Q=[[random.uniform(-1.0, 1.0),random.uniform(-1.0, 1.0),random.uniform(-1.0, 1.0),random.uniform(-1.0, 1.0),random.uniform(-1.0, 1.0)]]
val0=[]
val1=[]
val2=[]
val3=[]
val4=[]
temp=[]
time=[]
time1=[]
time2=[]
ran=4
tt=[]
for h in range(ran):
    val0=[]
    val1=[]
    val2=[]
    val3=[]
    val4=[]
    temp=[]

    t=[]
    #print(Q[h])
    for i in range(row):
        t1=(Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i]
        t2=((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])*final[1][i]
        t3=((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])*final[2][i]
        t4=((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])*final[3][i]
        t5=((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])*final[4][i]

        val0.append(t1)
        val1.append(t2)
        val2.append(t3)
        val3.append(t4)
        val4.append(t5)
    val=[]
    val.append(sum(val0)/row)
    val.append(sum(val1)/row)
    val.append(sum(val2)/row)
    val.append(sum(val3)/row)
    val.append(sum(val4)/row)

    for i in range(col):
        temp.append(float(Q[h][i]-(val[i]*step_size)))
    Q.append(list(temp))
    for i in range(row):

        t.append(((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])**2)
        tt.append(((Q[h][0]+Q[h][1]*final[1][i]+Q[h][2]*final[2][i]+Q[h][3]*final[3][i]+Q[h][4]*final[4][i])-final[0][i])**2)

    J.append(sum(t)/(2*row))
for i in range(row*ran):
    time2.append(i)
one=pd.Series(tt, index=time2)

#plt.show()

print(len(t))


#print(J)
time=[]
time1=[]
for i in range(ran):
    time.append(i)
Js = pd.Series(J, index=time)
for i in range(ran+1):
    time1.append(i)
Js = pd.Series(J, index=time)
print(J[len(J)-1])
K=[x[0] for x in Q]
Qs0 = pd.Series(K, index=time1)
K=[x[1] for x in Q]
print(K[len(K)-1])
Qs1 = pd.Series(K, index=time1)
K=[x[2] for x in Q]
print(K[len(K)-1])
Qs2 = pd.Series(K, index=time1)
K=[x[3] for x in Q]
print(K[len(K)-1])
Qs3 = pd.Series(K, index=time1)
K=[x[4] for x in Q]
print(K[len(K)-1])
Qs4 = pd.Series(K, index=time1)

plt.subplot(2, 1, 1)
one.plot()
plt.title('Cost Function J')
plt.ylabel('Normalized Value')

plt.subplot(2, 1, 2)
Js.plot()
Qs0.plot()
Qs1.plot()
Qs2.plot()
Qs3.plot()
Qs4.plot()
#plt.plot(x1, y1, 'ro-')
plt.title('J averaged over all inputs(B) all Q updates')
plt.ylabel('Normalized Value')
plt.show()
