import datetime as dt 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

from OdriveClass import *

# Note that positive velocity values lower winch 1
MAX_VEL = 100000 # Max. speed for winch in encoder counts per second

def animate(i, ys, odrv):
    time_end = time.time()+0.4
    graph_data_arr = []
    graph_data_arr.append(odrv0.get_current(0))
    while time.time() < time_end:
        graph_data_arr.append(odrv0.get_current(0))
    graph_data = np.mean(graph_data_arr)

    # Add x and y to lists
    #xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    #xs.append(i)
    ys.append(graph_data)

    # Limit x and y lists to 20 items
    #xs = xs[-20:]
    #ys = ys[-20:]
    ys = ys[-x_len:]
    
    # Format plot
    #ax.clear()
    #ax.plot(xs,ys)

    # Update line with new Y values
    line.set_ydata(ys)

    return line,

    # Format plot
    #plt.xticks(rotation=45, ha='right')
    #plt.subplots_adjust(bottom=0.30)
    #plt.title('Current over Time')
    #plt.ylabel('Current (A)')


odrv0 = Odrive('20673881304E') # Only has 1 winch
#odrv1 = Odrive('2087377E3548') # Has 2 winches

# Position control enabled

print(odrv0.axis[0].encoder.pos_estimate)
odrv0.PosMove(odrv0.axis[0].encoder.pos_estimate,0)
#raise SystemExit(0)

# Parameters
x_len = 200
y_range = [-3,1]

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
#xs = []
#ys = []
xs = list(range(0,200))
ys = [0] * x_len
ax.set_ylim(y_range)

line, = ax.plot(xs,ys)

plt.title('Current over Time')
plt.xlabel('Samples')
plt.ylabel('Current (A)')

# Set up plot to call animate() function periodically
#ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval =1000)
ani = animation.FuncAnimation(fig, animate, fargs=(ys, odrv0), interval =50, blit=True)
plt.show()

