import numpy as np

class PID():
    def __init__(self, kp=0.1, ki=0.1, kd=0.05, memory=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.memory = memory
        self.previous_error = 0
        self.error_accumulator = 0
        self.history = {'error':[], 'state':[]}
        
    def observe(self, error, state):
        """ records the convergence history"""
        self.history['error'].append(error)
        self.history['state'].append(state)
        
    def converge(self, state, target, dt=0.05):
        """ very basic PID implementation"""
            
        error = target - state
        
        self.error_accumulator += error
        
        p = self.kp * error
        i = self.ki * self.error_accumulator
        d = self.kd * (self.previous_error - error) / dt
        
        self.observe(error, state)
        self.previous_error = error
        
        return np.array(p+i+d)