import matplotlib.pyplot as plt
import numpy.random as rnd
from matplotlib.patches import Ellipse
import numpy as np


def plotEllipse(X, Y, W, H, A):
    NUM = 1 

    ells = [Ellipse(xy=[X,Y], width=W, height=H, angle=H)
            for i in range(NUM)]

    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    
    for e in ells:
        ax.add_artist(e)
        e.set_clip_box(ax.bbox)
        e.set_alpha(rnd.rand())
        e.set_facecolor(rnd.rand(3))

    return

def plotEllipse2(X,Y,W,H,A):
    ax=plt.gca()
    ellipse = Ellipse(xy=(X,Y), width=W, height=H,angle=A,fill=False)
    ax.add_patch(ellipse)
    return 0
plt.figure()
plt.axis([0,10,0,10])
plotEllipse2(0.5,0.5,0.25,0.125,0.4*360)
plotEllipse2(0.25,0.25,0.25,0.125,0.4*360)
plt.show()
