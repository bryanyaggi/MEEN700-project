from filterpy.kalman import ExtendedKalmanFilter
from filterpy.stats.stats import covariance_ellipse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from radio_localization.bicycle_model import BicycleModel
from radio_localization.radio_model import SPEED_OF_LIGHT, SECONDS_PER_100NS
from radio_localization.radio_model import RadioModel
from radio_localization.silvus_constants import INVALID_RSSI, INVALID_TOF

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
    def __init__(self, wheelbase, velocityVariance, steeringVariance):
        ExtendedKalmanFilter.__init__(self, dim_x=3, dim_z=1, dim_u=2)
        self.model = BicycleModel(wheelbase)
        self.Q = np.diag([1, 1, 1 * np.pi / 180])
        self.M = np.diag([velocityVariance, steeringVariance])
        self.wheelbase = wheelbase

    def predict(self, u, dt):
        self.F = np.array([
            [1, 0, -u[0] * np.sin(self.x[2]) * dt],
            [0, 1, u[0] * np.cos(self.x[2]) * dt],
            [0, 0, 1]
        ])
        self.x = self.model.stepKinematic(self.x, u, timestep=dt)
        
        # Calcualte Q from control noise and Jacobian: Q = VMV^T
        V = np.array([
            [np.cos(self.x[2]) * dt, 0],
            [np.sin(self.x[2]) * dt, 0],
            [np.tan(u[1]) * dt / self.wheelbase, u[0] * dt * (np.tan(u[1]) ** 2 + 1) / self.wheelbase]
        ])
        self.Q = np.dot(V, self.M).dot(V.T)
        
        self.P = np.dot(self.F, self.P).dot(self.F.T) + self.Q

class LocalizationFilter:
    def __init__(self, wheelbase, velocityVariance, steeringVariance,
            baseStationLocations, baseStationAs, baseStationNs,
            rssiVariance, tofVariance, imuVariance,
            rssiRangeThreshold=1e3, tofRangeThreshold=0,
            rssiVarianceFunctionOfRange=False, tofVarianceFunctionOfRange=True):
        '''
        baseStationLocations is a list of tuples of the form (x, y) specifying location in meters
        baseStationAs is a list of RSSI measurements at 1 meter
        '''
        self.baseStationLocations = np.array(baseStationLocations)
        self.baseStationAs = np.array(baseStationAs)
        self.baseStationNs = np.array(baseStationNs)

        self.radioModel = RadioModel(self.baseStationLocations, self.baseStationAs, self.baseStationNs,
                rssiVariance, tofVariance)

        self.createFilter(wheelbase, velocityVariance, steeringVariance)
        
        self.R_rssi = np.diag([rssiVariance]) # RSSI
        self.R_tof = np.diag([tofVariance]) # ToF
        self.R_imu = np.diag([imuVariance]) # radians

        self.rssiRangeThreshold = rssiRangeThreshold # meters
        self.tofRangeThreshold = tofRangeThreshold # meters

        self.rssiVarianceFunctionOfRange = rssiVarianceFunctionOfRange
        self.tofVarianceFunctionOfRange = tofVarianceFunctionOfRange

    def createFilter(self, wheelbase, velocityVariance, steeringVariance):
        #self.filter = ExtendedKalmanFilter(dim_x=3, dim_z=1)
        self.filter = LocalizationEkf(wheelbase, velocityVariance, steeringVariance)

        # initial estimate and covariance
        self.filter.x = np.array([0, 0, 0])
        self.filter.P = np.diag([1e6, 1e6, 1.])

    def hFunRssi(self, baseStationIndex):
        '''
        Returns function for h(x), which calculates the expected z given x
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        A = self.baseStationAs[baseStationIndex]
        n = self.baseStationNs[baseStationIndex]
        return lambda x: np.array([A - 10 * n * np.log10(np.sqrt((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2))])

    def hMatrixFunRssi(self, baseStationIndex):
        '''
        Returns function for H
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        n = self.baseStationNs[baseStationIndex]
        return lambda x: np.array([[
            -10 * n * (x[0] - xBs) / (((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2) * np.log(10)),
            -10 * n * (x[1] - yBs) / (((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2) * np.log(10)),
            0]])
    
    def hFunTof(self, baseStationIndex):
        '''
        Returns function for h(x), which calculates the expected z given x
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        return lambda x: np.array([np.sqrt((xBs - x[0]) ** 2 + (yBs- x[1]) ** 2) / (SPEED_OF_LIGHT * SECONDS_PER_100NS)])

    def hMatrixFunTof(self, baseStationIndex):
        '''
        Returns function for H
        '''
        xBs, yBs = self.baseStationLocations[baseStationIndex]
        return lambda x: np.array([[
            (x[0] - xBs) / ((SPEED_OF_LIGHT * SECONDS_PER_100NS) * np.sqrt((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2)),
            (x[1] - yBs) / ((SPEED_OF_LIGHT * SECONDS_PER_100NS) * np.sqrt((xBs - x[0]) ** 2 + (yBs - x[1]) ** 2)),
            0]])

    def incorporateRssiMeasurements(self, rssis):
        rssiRanges = self.radioModel.getRangesFromRssi(rssis)
        rssiVars = self.radioModel.getRssiVariances(rssis)
        for i in range(len(rssis)):
            if rssis[i] >= INVALID_RSSI: # skip if invalid RSSI
                continue
            h = self.hFunRssi(i)
            if rssiRanges[i] > self.rssiRangeThreshold: # skip if range too large
                continue
            H = self.hMatrixFunRssi(i)
            if self.rssiVarianceFunctionOfRange:
                R = np.array([[rssiVars[i]]])
            else:
                R = self.R_rssi
            print("Incorporating RSSI %d, RSSI: %f, range: %f var: %f" %(i, rssis[i], rssiRanges[i], R[0, 0]))
            self.filter.update(np.array([rssis[i]]), H, h, R=R)

    def incorporateTofMeasurements(self, tofs):
        tofRanges = self.radioModel.getRangesFromTof(tofs)
        tofVars = self.radioModel.getTofVariances(tofs)
        for i in range(len(tofs)):
            if tofs[i] <= INVALID_TOF: # skip if invalid ToF
                continue
            h = self.hFunTof(i)
            if tofRanges[i] < self.tofRangeThreshold: # skip if range too short
                continue
            H = self.hMatrixFunTof(i)
            if self.tofVarianceFunctionOfRange:
                R = np.array([[tofVars[i]]])
            else:
                R = self.R_tof
            print("Incorporating ToF %d, ToF: %f, range: %f, var: %f" %(i, tofs[i], tofRanges[i], R[0, 0]))
            self.filter.update(np.array([tofs[i]]), H, h, R=R)

    def incorporateImuMeasurement(self, yaw):
        h = lambda x: np.array([x[2]])
        H = lambda x: np.array([[0, 0, 1]])
        self.filter.update(np.array([yaw]), H, h, R=self.R_imu, residual=residualFunYaw)
    
    def run(self):
        rssiRanges = self.getRangesFromRssi()
        self.incorporateRssiMeasurements(rssiRanges)

'''
TODO: Fix how measurements are obtained
'''
class Test(unittest.TestCase):
    def testUpdateRssi(self):
        simulation = Simulation(baseStationLocations, baseStationAs, vehiclePose=vehiclePose)
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationAs = [20] * len(baseStationLocations)
        baseStationsNs = [3.0] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationAs)
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
        baseStationAs = [20] * len(baseStationLocations)
        baseStationsNs = [3.0] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationAs)
        l.plot()
        imuYaw = np.pi / 4
        l.incorporateImuMeasurement(imuYaw)
        l.plot()

    def testPredict(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationAs = [20] * len(baseStationLocations)
        baseStationsNs = [3.0] * len(baseStationLocations)
        l = Localization(baseStationLocations, baseStationAs)
        l.filter.P = np.diag([1, 1, np.pi / 180])
        u = np.array([10, 0])
        print('state: %s, uncertainty: %s' %(l.filter.x, l.filter.P))
        l.plot()
        l.filter.predict(u, dt=5)
        print('state: %s, uncertainty: %s' %(l.filter.x, l.filter.P))
        l.plot()
    
    def testPlot(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationAs = [20] * len(baseStationLocations)
        vehiclePose = (0, 0, np.pi/4)
        l = Localization(baseStationLocations, baseStationAs, vehiclePose=vehiclePose)
        rssiRanges = l.getRangesFromRssi()
        l.plot(rssiRanges)
