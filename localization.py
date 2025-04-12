from filterpy.kalman import ExtendedKalmanFilter
from filterpy.stats.stats import covariance_ellipse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from simulation import Simulation

import unittest

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
        self.filter = ExtendedKalmanFilter(dim_x=3, dim_z=1)

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

    def hFun(self, baseStationIndex):
        '''
        Returns function for h(x), which calculates the expected z given x
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        return lambda x: ((xBs - x[0]) ** 2 + (yBs- x[1]) ** 2) ** 0.5

    def hMatrixFun(self, baseStationIndex):
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
            h = self.hFun(i)
            H = self.hMatrixFun(i)
            self.filter.update(np.array([ranges[i]]), H, h, R=self.R_rssi)
            self.plot(ranges)

    def incorporateImuMeasurement(self, yaw):
        pass
    
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
        dx = l * np.cos(self.simulation.vehiclePose[2])
        dy = l * np.sin(self.simulation.vehiclePose[2])
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
        l.run()
    
    def testPlot(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        vehiclePose = (0, 0, np.pi/4)
        l = Localization(baseStationLocations, baseStationPowerTxs, vehiclePose=vehiclePose)
        rssiRanges = l.getRangesFromRssi()
        l.plot(rssiRanges)