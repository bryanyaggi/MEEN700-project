import math
import matplotlib.pyplot as plt
import numpy as np

import unittest

class BicycleModel:
    def __init__(self, wheelbase, plot=False):
        '''
        Reference point is center of rear wheel
        '''
        self.l = wheelbase
        self.plot = plot
        if plot:
            self.plotInit()

    @staticmethod
    def limitYaw(yaw):
        '''
        Limits yaw to range (-pi, pi]
        '''
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
    
    def stepKinematic(self, state, action, timestep=0.1):
        '''
        state is tuple (forward velocity, steering angle)
        '''
        v = action[0]
        delta = action[1]

        # kinematics
        xDot = v * math.cos(state[2])
        yDot = v * math.sin(state[2])
        thetaDot = v * math.tan(delta) / (self.l)
        stateDot = np.array([xDot, yDot, thetaDot])

        # calculate next state
        nextState = state + stateDot * timestep
        nextState[2] = self.limitYaw(nextState[2])

        if self.plot:
            self.plotPose(nextState, delta)

        return nextState
    
    def plotInit(self):
        plt.ion # interactive on
        fig, self.ax = plt.subplots()
        self.ax.set_title('Trajectory')
        self.ax.set_xlabel('meters')
        self.ax.set_ylabel('meters')
        self.ax.set_aspect('equal')
        plt.pause(0.1)

    def plotPose(self, pose, steerAngle):
        x = pose[0]
        y = pose[1]
        yaw = pose[2]

        #print((yaw + steer_angle) * 180 / math.pi)

        # vehicle axles
        xr = x
        yr = y
        xf = x + self.l * math.cos(yaw)
        yf = y + self.l * math.sin(yaw)

        self.ax.scatter(x, y, color='black') # body center
        self.ax.plot([xr, xf], [yr, yf], color='blue') # body
        self.ax.arrow(xf, yf, math.cos(yaw + steerAngle),
                math.sin(yaw + steerAngle), color='green') # front wheel vector
        plt.pause(0.1)
    
class Test(unittest.TestCase):
    def testKinematic(self):
        bm = BicycleModel(wheelbase=2.0, plot=True)
        pose = np.zeros(3)
        action = [10.0, math.pi / 18]
        for i in range(100):
            pose = bm.stepKinematic(pose, action)