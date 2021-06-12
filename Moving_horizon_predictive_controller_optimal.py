import numpy as np
from numpy.core.fromnumeric import repeat
from numpy.lib.polynomial import poly1d
from scipy.optimize import minimize, dual_annealing, shgo, differential_evolution, basinhopping
import matplotlib.pyplot as plt
import random
from matplotlib.animation import FuncAnimation
from scipy.signal import savgol_filter
import numba


def cost_function(params, final_pos, last_pos, black_hole_center):
    cost = 0;
    velocity_xy = np.zeros(shape = (len(params)//2, 2))
    current_pos = np.zeros(shape = (len(params)//2, 2))
    velocity_xy[:len(params)//2,0] = params[:len(params)//2];
    velocity_xy[:len(params)//2,1] = params[len(params)//2 :];
    d1 = 0
    d2 = 0
    init_pos = last_pos;
    
    for i in range(len(params)//2):
        current_pos[i] = init_pos + velocity_xy[i];
        rd1 = 1;
        d1 += ((current_pos[i][0]-black_hole_center[0]*rd1)**2)  + ((current_pos[i][1]-black_hole_center[0]*rd1)**2)
        d2 += ((current_pos[i][0]-final_pos[0])**2) + ((current_pos[i][1]-final_pos[1])**2)
       
    cost1 = 160*np.exp((-0.019*(d1)));
    cost2 = -200*np.exp(-0.00009*(d2));
    cost3 = -100*np.exp(-0.009*(d2));

    cost =  (cost1 + cost2 + cost3); # we're searching for the horizon of minimum cost.
    #print(cost1, cost2, cost3)
    return cost

class Predictive_control:
    def __init__(self, horizon):
        self.init_pos = np.array([0,0], np.float32); # np.zeros(2), could be.
        self.target_pos = np.array([19,19], np.float32);
        self.d = 0;
        self.black_hole_center = np.array([7.5,7.5], np.float32);
        self.horizon = horizon;
        self.history = list();
        self.horizon_history = list()
        
    def get_trajectory(self):
        last_pos = self.init_pos;
        black_hole_center = self.black_hole_center;
        final_pos = self.target_pos;
        self.trajectory = list();
        x = list();
        y = list();
        max_velocity = [[-0.075,0.075]]*self.horizon*2;
        params = np.zeros(self.horizon*2)
        max_velocity = np.array(max_velocity)
        assert len(max_velocity) == self.horizon*2
        offset = 3
        counter = 0
        while np.sum(abs(last_pos - final_pos)) >= 0.5:
            counter += 1
            x.append(last_pos[0])
            y.append(last_pos[1])
            #if counter < 5 or len(np.unique(self.history[-3:])) <= 2:
            #    sol = dual_annealing(cost_function, max_velocity , args = (final_pos, last_pos,black_hole_center), maxiter = 15); #, maxiter = 50
            #else:
            sol = minimize(cost_function, params , args = (final_pos, last_pos,black_hole_center), bounds = max_velocity, method = 'SLSQP');
                                                 
            params = sol.x;
            self.horizon_history.append(params)
            last_pos += np.array([sol.x[0+offset], sol.x[len(params)//2 + offset]]); # we take a 3 step increment toward the horizon.
            self.history.append(last_pos.flatten())
            print(last_pos, sol.fun)
            print(np.sum(abs(last_pos - final_pos)))
            #print(last_pos)
        z = np.polyfit(x,y, 4);
        p = np.poly1d(z);
        
        self.i = -1;
        fig, ax = plt.subplots()
        xdata, ydata , ydata2 = [], [], []
        plt.xlabel("X Coordinates")
        plt.ylabel("Y Coordinates")
        ln, = plt.plot([], [], 'ro', alpha = 0.25)
        ln2, = plt.plot([],[], 'b*', alpha = 0.15)
        ln3, = plt.plot([], [], marker = "^", markersize = 60, alpha = 0.49)
        ln3.set_data([self.black_hole_center[0]], [self.black_hole_center[1]])
        

        def init():
            ax.set_xlim(-5, 21)
            ax.set_ylim(-5, 21)
            #ln.set_data(9,9, markersize = 2)
            return ln,ln2,ln3,

        def update(frame):
            self.i += 1
            try:
                xdata.append(x[self.i])
                if self.i == 0 or self.i == len(x)-1:
                    ydata.append(y[self.i])
                else:
                    ydata.append(p(x[self.i]))
                ydata2.append(y[self.i])
                ln.set_data(xdata, ydata2)
                ln2.set_data(xdata, ydata)
                
                
            except:
                pass
            return ln,ln2,ln3,

        ani = FuncAnimation(fig, update, frames=range(len(x)), interval = 10,
                    init_func=init, blit=True, repeat = True)
        plt.show()

controller = Predictive_control(10);
controller.get_trajectory();