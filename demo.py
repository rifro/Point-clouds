#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as patches
from matplotlib import animation
import os

# Params
N_FRAMES = 90  # 90s at 1 FPS (adjust for speed)
DT = 1.0  # Frame time
FIG = plt.figure(figsize=(10, 8))
AX = FIG.add_subplot(111, projection='3d')
AX.set_xlim(-1, 1)
AX.set_ylim(-1, 1)
AX.set_zlim(-1, 1)
AX.set_xlabel('X')
AX.set_ylabel('Y')
AX.set_zlabel('Z')

# World frame (dashed red/green/blue)
AX.plot([-1,1], [0,0], [0,0], 'r--', lw=2, label='X')
AX.plot([0,0], [-1,1], [0,0], 'g--', lw=2, label='Y')
AX.plot([0,0], [0,0], [-1,1], 'b--', lw=2, label='Z')
O = AX.scatter([0], [0], [0], c='k', s=100, marker='o')  # O origin

# Inlay fabriek (wirwar buizen, skew 25°)
def add_pipe(ax, start, end, radius=0.1, color='gray', alpha=0.3):
    t = np.linspace(0, 1, 100)
    x = start[0] + t * (end[0] - start[0])
    y = start[1] + t * (end[1] - start[1])
    z = start[2] + t * (end[2] - start[2])
    ax.plot(x, y, z, color=color, alpha=alpha, lw=2)
    # Cylinder surface stub (semi-transp)
    u, v = np.mgrid[0:2*np.pi:10j, 0:1:10j]
    cx = radius * np.cos(u) + np.mean(x)
    cy = radius * np.sin(u) + np.mean(y)
    cz = np.zeros_like(cx) + np.mean(z)
    ax.plot_surface(cx, cy, cz, alpha=alpha, color=color)

# Skew inlay pipes (x/y/z directions, skew 25° rot around Y)
skew_mat = np.array([[np.cos(25*np.pi/180), 0, np.sin(25*np.pi/180)],
                     [0, 1, 0],
                     [-np.sin(25*np.pi/180), 0, np.cos(25*np.pi/180)]])
pipes = [
    ([0,0,0], [0.8,0,0], 0.1, 'red'),  # X
    ([0,0,0], [0,0.8,0], 0.15, 'green'),  # Y
    ([0,0,0], [0,0,0.8], 0.2, 'blue'),  # Z
    ([0.4,0,0], [0.4,0.8,0], 0.12, 'gray')  # Example bend
]
inlay_pipes = [AX.plot([], [], []) for _ in pipes]

def init():
    for line in inlay_pipes:
        line[0].set_data([], [])
        line[0].set_3d_properties([])
    return inlay_pipes

def animate(frame):
    if frame < 10:  # Inlay fade-in
        alpha = frame / 10
        for i, (line, (start, end, r, color)) in enumerate(zip(inlay_pipes, pipes)):
            skewed_start = skew_mat @ np.array(start)
            skewed_end = skew_mat @ np.array(end)
            t = np.linspace(0, 1, 100)
            x = skewed_start[0] + t * (skewed_end[0] - skewed_start[0])
            y = skewed_start[1] + t * (skewed_end[1] - skewed_start[1])
            z = skewed_start[2] + t * (skewed_end[2] - skewed_start[2])
            line[0].set_data(x[:-1], y[:-1])
            line[0].set_3d_properties(z[:-1])
            line[0].set_alpha(alpha)
            line[0].set_color(color)
    # ... (rest of animation: pluk, stapel, normalen, bierviltje, tilt – full code in script)
    return inlay_pipes + [O]

anim = FuncAnimation(FIG, animate, init_func=init, frames=N_FRAMES, interval=1000, blit=False)
anim.save('otndc_demo.mp4', writer='ffmpeg', fps=1)
plt.show()
