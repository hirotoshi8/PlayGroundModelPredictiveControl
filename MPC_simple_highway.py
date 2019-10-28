# Model Predictive Control Sample code in Udemy
# This software needs simulator provided by udemy MPC course
# The MPC targets the speed control such as high way scenario

# coding: utf-8

# In[2]:

import numpy as np
import sys
sys.path.insert(0, 'E:\WorkSpaceMPC\mpc-course-assignments')
from sim.sim1d import sim_run


# In[6]:

options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = True


# In[41]:

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2
        self.reference = [50,0,0] # x, y, v
        
    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0] # current positoin 
        v_t = prev_state[3] # current speed
        # Coding
        convert_pedal_to_acc = 1
        x_t_1 = x_t + v_t * dt - v_t/25 # predicted position
        v_t_1 = v_t + convert_pedal_to_acc * pedal # predicted speed
        
        return [x_t_1, 0, 0, v_t_1]
    
    def cost_function(self, u, *args):
        state = args[0]
        ref    = args[1]
        cost   = 0
        
        # Coding
        x_goal = ref[0]
        for k in range(0, self.horizon):
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            x_predicted = state[0]
        
            cost += (x_goal - x_predicted)**2
            
            speed_kmh = state[3]*3.6
            if( (speed_kmh) > 10.0):
                cost += 1000
            
        return cost



# In[42]:

sim_run(options, ModelPredictiveControl)


# In[ ]:




# In[ ]:



