import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def plt_vive_data(axis, filename, color):
    pullData = open(filename,"r").read()
    print(len(pullData))
    dataArray = pullData.split('\n')
    xar = []
    yar = []
    for eachLine in dataArray:
        if len(eachLine)>1:
            x,y,z = eachLine.split(',')
            xar.append(float(x))
            yar.append(float(z))
    axis.plot(xar,yar, c=color, label=filename)

    
plt_vive_data(ax1, "./left.csv", "red")
plt_vive_data(ax1, "./right.csv", "blue")

plt.legend()
plt.show()
