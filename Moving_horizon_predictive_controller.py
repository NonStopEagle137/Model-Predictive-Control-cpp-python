import numpy as np
from scipy.optimize import minimize, dual_annealing, shgo, differential_evolution, basinhopping
import matplotlib.pyplot as plt
import random
from matplotlib.animation import FuncAnimation
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
       
    cost1 = 20*np.exp((-0.09*(d1)));
    cost2 = -400*np.exp(-0.00009*(d2));
    cost3 = -40*np.exp(-0.009*(d2));

    cost =  (cost1 + cost2 + cost3); # we're searching for the horizon of minimum cost.
    
    return cost

class Predictive_control:
    def __init__(self, horizon):
        self.init_pos = np.array([0,0], np.float32); # np.zeros(2), could be.
        self.target_pos = np.array([19,19], np.float32);
        self.d = 0;
        self.black_hole_center = np.array([5.5,5.5], np.float32);
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
        max_velocity = [[-0.15,0.15]]*self.horizon*2;
        params = np.zeros(self.horizon*2)
        max_velocity = np.array(max_velocity)
        assert len(max_velocity) == self.horizon*2
        offset = 3
        counter = 0
        while np.sum(abs(last_pos - final_pos)) >= 0.5:
            counter += 1
            x.append(last_pos[0])
            y.append(last_pos[1])
            if counter < 5 or len(np.unique(self.history[-3:])) <= 2:
                sol = differential_evolution(cost_function, max_velocity , args = (final_pos, last_pos,black_hole_center)); #, maxiter = 50
            else:
                sol = minimize(cost_function, params , args = (final_pos, last_pos,black_hole_center), bounds = max_velocity, method = 'SLSQP');
                                                 
            params = sol.x;
            self.horizon_history.append(params)
            last_pos += np.array([sol.x[0+offset], sol.x[len(params)//2 + offset]]); # we take a 3 step increment toward the horizon.
            self.history.append(last_pos.flatten())
            print(last_pos, sol.fun)
            #print(last_pos)
        
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

        ani = FuncAnimation(fig, update, frames=range(len(x)), interval = 100,
                    init_func=init, blit=True)
        plt.show()

controller = Predictive_control(10);
controller.get_trajectory();