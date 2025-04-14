from filterpy.kalman import ExtendedKalmanFilter
from filterpy.stats.stats import covariance_ellipse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from simulation import Simulation
from bicycle_model import BicycleModel

import unittest
    
def residualFunYaw(a, b):
    '''
    Keeps residual in range (-pi, pi]
    '''
    y = a - b
    y[0] %= (2 * np.pi)
    if y[0] > np.pi:
        y[0] -= (2 * np.pi)
    return y

class LocalizationEkf(ExtendedKalmanFilter):
    def __init__(self, wheelbase, velStdDev=0.1, steerStdDev=np.pi / 180):
        ExtendedKalmanFilter.__init__(self, dim_x=3, dim_z=1, dim_u=2)
        self.model = BicycleModel(wheelbase)
        #self.velStdDev = velStdDev
        #self.steerStdDev = steerStdDev
        self.Q = np.diag([1, 1, 2 * np.pi / 180])

    def predict(self, u, dt):
        self.F = np.array([
            [1, 0, -u[0] * np.sin(self.x[2]) * dt],
            [0, 1, u[0] * np.cos(self.x[2]) * dt],
            [0, 0, 1]
        ])
        self.x = self.model.stepKinematic(self.x, u, timestep=dt)
        # TODO calcualte Q from control noise and Jacobian: Q = VMV^T
        self.P = np.dot(self.F, self.P).dot(self.F.T) + self.Q

class Localization:
    def __init__(self, baseStationLocations, baseStationPowerTxs, rssiN=2, speedSound=343, vehiclePose=(0, 0, 0)):
        '''
        baseStationLocations is a list of tuples of the form (x, y) specifying location in meters
        baseStationPowerTxs is a list of transmit powers at 1 meter
        '''
        self.simulation = Simulation(baseStationLocations, baseStationPowerTxs, vehiclePose=vehiclePose)
        
        self.baseStationLocations = np.array(baseStationLocations)
        self.baseStationPowerTxs = np.array(baseStationPowerTxs)
        self.rssiN = rssiN
        self.speedSound = speedSound

        self.createFilter()

    def createFilter(self):
        #self.filter = ExtendedKalmanFilter(dim_x=3, dim_z=1)
        self.filter = LocalizationEkf(wheelbase=2.0)

        # initial estimate and covariance
        self.filter.x = np.array([0, 0, 0])
        self.filter.P = np.diag([1e6, 1e6, 1.])

        self.R_rssi = np.diag([1e3]) # RSSI
        self.R_imu = np.diag([np.pi / 180]) # radians

    def getRssiMeasurements(self):
        return self.simulation.getRssiMeasurements()

    def getRangesFromRssi(self):
        rssiMeasurements = self.getRssiMeasurements()
        ranges = 10 ** ((self.baseStationPowerTxs - rssiMeasurements) / (10 * self.rssiN))
        return ranges

    def hFunRssi(self, baseStationIndex):
        '''
        Returns function for h(x), which calculates the expected z given x
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        return lambda x: np.array([((xBs - x[0]) ** 2 + (yBs- x[1]) ** 2) ** 0.5])

    def hMatrixFunRssi(self, baseStationIndex):
        '''
        Returns function for H
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        return lambda x: np.array([[
            (x[0] - xBs) / ((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2) ** 0.5,
            (x[1] - yBs) / ((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2) ** 0.5,
            0]])

    def incorporateRssiMeasurements(self, ranges):
        for i in range(len(ranges)):
            h = self.hFunRssi(i)
            H = self.hMatrixFunRssi(i)
            self.filter.update(np.array([ranges[i]]), H, h, R=self.R_rssi)
            self.plot(ranges) 

    def incorporateImuMeasurement(self, yaw):
        h = lambda x: np.array([x[2]])
        H = lambda x: np.array([[0, 0, 1]])
        self.filter.update(np.array([yaw]), H, h, R=self.R_imu, residual=residualFunYaw)
    
    def plotInit(self):
        pass

    def plot(self, rangesRssi=None):
        fig, ax = plt.subplots()

        # ground truth pose
        l = 1e-4
        dx = l * np.cos(self.simulation.vehiclePose[2])
        dy = l * np.sin(self.simulation.vehiclePose[2])
        ax.scatter(self.simulation.vehiclePose[0], self.simulation.vehiclePose[1], color='black', label='vehicle pose')
        ax.quiver(self.simulation.vehiclePose[0], self.simulation.vehiclePose[1], dx, dy, units='inches', angles='xy')

        # estimated pose
        dx = l * np.cos(self.filter.x[2])
        dy = l * np.sin(self.filter.x[2])
        ax.scatter(self.filter.x[0], self.filter.x[1], color='green', label='vehicle pose')
        ax.quiver(self.filter.x[0], self.filter.x[1], dx, dy, units='inches', angles='xy', color='green')
        # covariance ellipse
        orientation, width, height = covariance_ellipse(self.filter.P[:2, :2])
        ellipse = patches.Ellipse(self.filter.x[:2], width, height, angle=orientation * 180/np.pi,
                                  edgecolor='green', fill=False)
        ax.add_patch(ellipse)

        # base stations
        ax.scatter(self.baseStationLocations[:, 0], self.baseStationLocations[:, 1], color='blue', label='base stations')
        # RSSI ranges
        if rangesRssi is not None: 
            for i in range(len(rangesRssi)):
                circle = patches.Circle((self.baseStationLocations[i][0], self.baseStationLocations[i][1]), rangesRssi[i],
                                        edgecolor='blue', fill=False)
                ax.add_patch(circle)
        
        ax.set_xlabel('x [meters]')
        ax.set_ylabel('y [meters]')
        #ax.legend()
        ax.set_aspect('equal')
        plt.show()

    def run(self):
        rssiRanges = self.getRangesFromRssi()
        self.incorporateRssiMeasurements(rssiRanges)

class Test(unittest.TestCase):
    def testUpdateRssi(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationPowerTxs)
        l.plot()
        rssiRanges = l.getRangesFromRssi()
        l.incorporateRssiMeasurements(rssiRanges)

    def testResidualYaw(self):
        a = 0
        bs = np.arange(9/4 * np.pi, step=np.pi / 4)
        print(bs)
        for b in bs:
            r = residualFunYaw(a, b)
            print('%s - %s = %s' %(a, b, r))

    def testUpdateImu(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationPowerTxs)
        l.plot()
        imuYaw = np.pi / 4
        l.incorporateImuMeasurement(imuYaw)
        l.plot()

    def testPredict(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationPowerTxs)
        l.filter.P = np.diag([1, 1, np.pi / 180])
        u = np.array([10, 0])
        print('state: %s, uncertainty: %s' %(l.filter.x, l.filter.P))
        l.plot()
        l.filter.predict(u, dt=5)
        print('state: %s, uncertainty: %s' %(l.filter.x, l.filter.P))
        l.plot()
    
    def testPlot(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        vehiclePose = (0, 0, np.pi/4)
        l = Localization(baseStationLocations, baseStationPowerTxs, vehiclePose=vehiclePose)
        rssiRanges = l.getRangesFromRssi()
        l.plot(rssiRanges)