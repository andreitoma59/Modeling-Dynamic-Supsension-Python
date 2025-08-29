from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib
import numpy as np
import tkinter
matplotlib.use('TkAgg')
class Visualizer(object):
    def __init__(self, callback=None, interval=100.0, simulation_time=20.0, initial=(0, 0, 0, 0, 0, 0),displ_list=None,yr=None,acc_list=None,x0r=None):
        # Set loop interval
        self.interval = interval
        self.simulation_time = simulation_time
        self.maxr=max(yr)
        self.maxd=max(displ_list)
        self.maxa=max(acc_list)
        self.mina=min(acc_list)
        # Create figure
        self.fig = plt.figure(1, figsize=(16, 8))
        # Create animation plot area
        self.animation_plot = self.fig.add_subplot(223)
        # Create position plot
        self.position_plot = self.fig.add_subplot(221)
        self.position_plot.set_xlim((-0.6, self.simulation_time+2))
        self.position_plot.set_ylim((0, self.maxr+1))
        self.position_plot_patch = GraphPlot()
        self.position_plot.add_patch(self.position_plot_patch.first_patch)
        self.position_plot.add_patch(self.position_plot_patch.second_patch)
        self.position_plot.text(0.8, 0.8, 'Blue: Road', transform=self.position_plot.transAxes)
        self.position_plot.text(0.03, 0.9, 'Road Profile Plot', transform=self.position_plot.transAxes)
        # Create velocity plot
        self.velocity_plot = self.fig.add_subplot(222)
        self.velocity_plot.set_xlim((-0.6, self.simulation_time+2))
        self.velocity_plot.set_ylim((-0.6, self.maxd+0.3))
        self.velocity_plot_patch = GraphPlot()
        self.velocity_plot.add_patch(self.velocity_plot_patch.first_patch)
        self.velocity_plot.add_patch(self.velocity_plot_patch.second_patch)
        self.velocity_plot.text(0.52, 0.8, 'Red:  Displacement without PID\nBlue: Displacement with PID', transform=self.velocity_plot.transAxes)
        self.velocity_plot.text(0.03, 0.9, 'Displacement Plot', transform=self.velocity_plot.transAxes)
        # Create acceleration plot
        self.acceleration_plot = self.fig.add_subplot(224)
        self.acceleration_plot.set_xlim((-0.6, self.simulation_time+2))
        self.acceleration_plot.set_ylim((self.mina, self.maxa+40))
        self.acceleration_plot_patch = GraphPlot()
        self.acceleration_plot.add_patch(self.acceleration_plot_patch.first_patch)
        self.acceleration_plot.add_patch(self.acceleration_plot_patch.second_patch)
        self.acceleration_plot.text(0.54, 0.8, 'Red:  Acceleration without PID\nBlue: Acceleration with PID', transform=self.acceleration_plot.transAxes)
        self.acceleration_plot.text(0.03, 0.9, 'Acceleration Plot', transform=self.acceleration_plot.transAxes)
        # Set axes limit
        self.animation_plot.set_xlim((-20, 20))
        self.animation_plot.set_ylim((0, 30))
        # Create animation core
        self.animate = animation.FuncAnimation(self.fig, self._animate, interval=self.interval,
                                               blit=True, repeat=True, init_func=self.init_func,save_count=1200)
        
        # Set initials to zero
        self.sim_time = 0
        self.x = initial[0]
        self.y = initial[1]
        self.vx = initial[2]
        self.vy = initial[3]
        self.ax = initial[4]
        self.ay = initial[5]
        # Set callback
        self.cb_func = callback
        plt.show(block=False)
        

    def init_func(self):
        # Set initials to zero
        pos_plot = self.position_plot_patch.init_func()
        vel_plot = self.velocity_plot_patch.init_func()
        acc_plot = self.acceleration_plot_patch.init_func()
        ret_ =acc_plot + vel_plot + pos_plot
        # return the objects to be redrawn
        return ret_

    def _animate(self, i):
        # update simulation time, i = n'th time that the method has
        # called, thus each represents self.interval ms of delay.
        self.sim_time = float(i) / (1000.0 / self.interval)
        # Update time

        # Get object xy from callback
        (x_int, road,displ,displ_pid,acc,acc_pid,z0,z1,z2,t,x,road_x,road_z,umf) = self.cb_func(self.sim_time,i)
        # Set the position of object
        vx = displ
        vy = displ_pid

        self.ax = acc
        self.ay = acc_pid
        self.vx = vx
        self.vy = vy
        self.x = x_int
        self.y = 15
       
        pos_plot = self.position_plot_patch.update([[self.sim_time, 0], [self.sim_time, road]])
        vel_plot = self.velocity_plot_patch.update([[self.sim_time, vx], [self.sim_time, vy]])
        acc_plot = self.acceleration_plot_patch.update([[self.sim_time, self.ax], [self.sim_time, self.ay]])
        
        #Geometric suspension parameters:
        h1 = 0.35  #resting position of unsprung cm
        h2 = 1.1   #resting position of sprung cm
        h3 = 0.2   #height of unsprung mass block
        h4 = 0.35  #height of sprung mass block
        w1 = 0.4   #width of unsprung mass block
        w2 = 0.5   #width of sprung mass block
        w3 = 0.1   #width of tire spring
        w4 = 0.15  #width of suspension spring
        w5 = 0.25  #spring/damper spacing
        fw = 1.2   #half of figure width
        x0_r = z0  #tire spring base position
        x0_s = (h1 + z1) + (h3 / 2) #suspension spring base position
        x0_t = (h1 + z1) - (h3 / 2) #unsprung mass block base position
        x0_b = (h2 + z2) - (h4 / 2) #spring mass block base position
        L1 = x0_t - x0_r #tire spring length
        L2 = x0_b - x0_s #suspension spring length
        dx = road_x[1] - road_x[0]
        xstart = max(x- fw, 0)
        istart = np.argmin(np.abs(xstart - road_x))+1
        xend = x + fw
        iend = np.argmin(np.abs(xend - road_x))+1
        xpstart = xstart - x
        xpend = fw
        xp = np.arange(xpstart,xpend+dx,dx) 
        zp = road_z[istart+1:iend] * umf
        maxi = min(len(xp), len(zp))
        self.animation_plot.clear()
        self.animation_plot.text(fw / 2, 2.5, f"{t:.3f} sec") #Display current simulation time
        self.animation_plot.plot(xp[:maxi], zp[:maxi], "k-") #Plot road profile 

        x0t = [0, x0_t]
        x1t = [(-w1) / 2, x0_t]
        x2t = [(-w1) / 2, h3+x0_t]
        x3t = [w1 / 2, h3+x0_t]
        x4t = [w1 / 2, x0_t]
        self.animation_plot.fill([x1t[0], x2t[0], x3t[0], x4t[0]],
                [x1t[1], x2t[1], x3t[1], x4t[1]], color=(65/255, 105/255, 225/255))#DREPTUNGHIUL DE JOS

        self.animation_plot.axis([-fw, fw, -0.25, 2.5],)
        x0b = [0, x0_b]
        x1b = [(-w2) / 2, x0_b]
        x2b = [(-w2) / 2, h4+x0_b]
        x3b = [w2 / 2, h4+x0_b]
        x4b = [w2 / 2, x0_b]
        self.animation_plot.fill([x1b[0], x2b[0], x3b[0], x4b[0]],
                [x1b[1], x2b[1], x3b[1], x4b[1]], color=(65/255, 105/255, 225/255))#DREPTUNHIUL DE SUS
        x0r = [0, x0_r]
        self.animation_plot.plot([x0r[0]], [x0r[1]], "ko", markersize=10, markerfacecolor="k")#roata de jos

        u = L1 / 9
        x1r = [0, u+x0_r]
        x2r = [(-w3) / 2, ((3 / 2) * u)+x0_r]
        x3r = [x2r[0] + w3,x2r[1]+u]
        x4r = [x3r[0] - w3,x3r[1]+u]
        x5r = [x4r[0] + w3,x4r[1]+u]
        x6r = [x5r[0] - w3,x5r[1]+u]
        x7r = [x6r[0] + w3,x6r[1]+u]
        x8r = [x7r[0] - w3,x7r[1]+u]
        x9r = [x8r[0] + w3 / 2,x8r[1]+ u / 2]
        x10r = [x9r[0],x9r[1]+u]
        self.animation_plot.plot([x[0] for x in [x0r, x1r, x2r, x3r, x4r, x5r, x6r, x7r, x8r, x9r, x10r]],
                [x[1] for x in [x0r, x1r, x2r, x3r, x4r, x5r, x6r, x7r, x8r, x9r, x10r]], "k-", linewidth=2)#ARCUL DE LA ROATA
        
        x0s = [(-w5) / 2, x0_s]
        u=L2/9
        x1s = [x0s[0]+0,x0s[1]+u]
        x2s = [x0s[0]+(-w4) / 2, x0s[1]+(3 / 2) * u]
        x3s = [x2s[0] + w4,x2s[1] + u]
        x4s = [x3s[0] - w4,x3s[1] + u]
        x5s = [x4s[0] + w4,x4s[1] + u]
        x6s = [x5s[0] - w4,x5s[1] + u]
        x7s = [x6s[0] + w4,x6s[1] + u]
        x8s = [x7s[0] - w4,x7s[1] + u]
        x9s = [x8s[0] + w4 / 2,x8s[1] + u / 2]
        x10s = [x9s[0],x9s[1] + u]
        self.animation_plot.plot([x[0] for x in [x0s, x1s, x2s, x3s, x4s, x5s, x6s, x7s, x8s, x9s, x10s]],
                [x[1] for x in [x0s, x1s, x2s, x3s, x4s, x5s, x6s, x7s, x8s, x9s, x10s]], "k-", linewidth=3)#ARCUL DE LA SUSPENSIE
        
        #PLOT SUSPENSION DAMPER
        x0d = [w5 / 2, x0_s]
        a = 0.7 * (h2 - h1 - h3 / 2 - h4 / 2)
        b = L2 - a
        c = 0.3 * w4
        x1d = [x0d[0]-c,x0d[1]+a]
        x2d = [x0d[0]-c,x0d[1]+ 0]
        x3d = [x0d[0]+c,x0d[1]+ 0]
        x4d = [x0d[0]+c,x0d[1]+ a]
        x5d = [x0d[0]-c,x0d[1]+ b]
        x6d = [x0d[0]+c,x0d[1]+ b]
        x7d = [x0d[0],x0d[1]+ L2]
        x8d = [x0d[0],x0d[1]+ b]
        self.animation_plot.plot([x[0] for x in [x1d, x2d, x3d, x4d]],
                [x[1] for x in [x1d, x2d, x3d, x4d]], "k-", linewidth=2)#DAMPERUL IN SINE
        self.animation_plot.plot([x[0] for x in [x5d, x6d]], [x[1] for x in [x5d, x6d]], "k-", linewidth=4)#PARTEA DIN JOS A DAMPERULUI
        self.animation_plot.plot([x[0] for x in [x7d, x8d]], [x[1] for x in [x7d, x8d]], "k-", linewidth=2)#BRATUL DIN DAMPER
        self.fig.canvas.draw()
        # return the objects to be redrawn
        ret_ = acc_plot + vel_plot + pos_plot
        return ret_


class GraphPlot():
    def __init__(self):
        self.first_patch = plt.Polygon([[0, 0]], closed=None, fill=None, edgecolor='r',ls='--')
        self.second_patch = plt.Polygon([[0, 0]], closed=None, fill=None, edgecolor='b')

    def init_func(self):
        self.first_patch.xy = [[0, 0]]
        self.second_patch.xy = [[0, 0]]
        return [self.first_patch, self.second_patch]

    def update(self, data):
        # Add current x position to x graph
        arr = self.first_patch.get_xy().tolist()
        arr.append(data[0])
        self.first_patch.xy = arr

        # Add current y position to y graph
        arr = self.second_patch.get_xy().tolist()
        arr.append(data[1])
        self.second_patch.xy = arr
        return [self.first_patch, self.second_patch]