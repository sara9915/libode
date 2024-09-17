from os.path import join
import numpy as np
from numpy import pi, nan, full, array, fromfile, sin, cos
import matplotlib.pyplot as pp
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle

#load results from files x.txt, where the values are arrangend in columns 
x = np.loadtxt('x.txt')
theta = np.loadtxt('theta.txt')
xdot = np.loadtxt('xdot.txt')
thetadot = np.loadtxt('thetadot.txt')



# L = 1.5
# axa = plt.subplot2grid((2,4), (0,0), colspan=2, rowspan=2)
# axa.set_xlim(-L, L)
# axa.set_ylim(-L, L)
# axa.set_aspect('equal')
# axa.set_xlabel('$x$')
# axa.set_ylabel('$t$')
# axa.set_title('Inverted Pendulum')


fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-0.5, 2))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.3
cart_height = 0.2

def init():
    line.set_data([], [])
    time_text.set_text('')
    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, patch


def animate(i):
    L = 0.3
    pxs = L*sin(3.14 + theta[i]) + x[i]
    pys = L*cos(theta[i])
    thisx = [x[i], pxs]
    thisy = [0, pys]

    
    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*1e-4))
    patch.set_x(x[i] - cart_width/2)
    return line, time_text, patch


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(x)),
                              interval=1, blit=True, init_func=init)

pp.show()
