#%%
import numpy as np
from scipy.optimize import minimize, dual_annealing, shgo, differential_evolution, basinhopping
import matplotlib.pyplot as plt
import random
from matplotlib.animation import FuncAnimation
import numba
@numba.jit (nopython = True)
def cost_function(params, final_pos, last_pos, black_hole_center):
    cost = 0;
    
    velocity_xy = params;
    
    
    current_pos = last_pos + velocity_xy;
    rd1 = (1+random.uniform(-0.05,0.05));
    rd2 = (1+random.uniform(-0.5,0.5))
    cost1 = 50*np.exp((-0.9*((current_pos[0]-black_hole_center[0]*rd1)**2+(current_pos[1]-black_hole_center[0]*rd1)**2)));
    cost2 = -90*np.exp(-0.019*((current_pos[0]-final_pos[0])**2 + (current_pos[1]-final_pos[1])**2))
    
    cost =  cost1 + cost2;
    return cost
    

class Predictive_control:
    def __init__(self):
        self.init_pos = np.array([0,0], np.float32); # np.zeros(2), could be.
        self.target_pos = np.array([19,19], np.float32);
        self.d = 0;
        self.black_hole_center = np.array([5.5,5.5], np.float32);
        
        
    def get_trajectory(self):
        last_pos = self.init_pos;
        black_hole_center = self.black_hole_center;
        final_pos = self.target_pos;
        self.trajectory = list();
        x = list();
        y = list();
        max_velocity = np.array([[-0.56,0.56], [-0.56,0.56]])
        while np.sum(abs(last_pos - final_pos)) >= 0.5:
            x.append(last_pos[0])
            y.append(last_pos[1])
            
            sol = dual_annealing(cost_function, max_velocity, args = (final_pos, last_pos,black_hole_center));
                                                         
            #params[0] = np.sqrt(sol.x[0]**2 +sol.x[1]**2)*(sol.x[0] - last_pos[0])/(sol.x[1] - last_pos[1])
            last_pos += sol.x;
            print(last_pos)
        
        self.i = -1;
        fig, ax = plt.subplots()
        xdata, ydata = [], []
        plt.xlabel("X Coordinates")
        plt.ylabel("Y Coordinates")
        ln, = plt.plot([], [], 'ro')
        

        def init():
            ax.set_xlim(-2, 21)
            ax.set_ylim(-2, 21)
            #ln.set_data(9,9, markersize = 2)
            return ln,

        def update(frame):
            self.i += 1
            try:
                xdata.append(x[self.i])
                ydata.append(y[self.i])
                ln.set_data(xdata, ydata)
            except:
                pass
            return ln,

        ani = FuncAnimation(fig, update, frames=range(len(x)),
                    init_func=init, blit=True)
        plt.show()

controller = Predictive_control();
controller.get_trajectory();
# %%
