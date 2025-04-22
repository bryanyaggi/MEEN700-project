#!/usr/bin/env python3

import numpy as np

from bicycle_model import BicycleModel

import unittest

class Simulation:
    def __init__(self, baseStationLocations, baseStationPowerTxs, rssiN=2, rssiStdDev=0.5, speedSound=343,
                 totStdDev=1e-3, wheelbase=2.0, vehiclePose=(0, 0, 0)):
        '''
        baseStationLocations is a list of tuples of the form (x, y) specifying location in meters
        baseStationPowerTxs is a list of transmit powers at 1 meter
        vehiclePose is a tuple of (x, y, yaw) in meters and radians
        '''
        self.baseStationLocations = np.array(baseStationLocations)
        self.baseStationPowerTxs = np.array(baseStationPowerTxs)
        self.rssiN = rssiN
        self.rssiStdDev = rssiStdDev
        self.speedSound = speedSound
        self.totStdDev = totStdDev
        self.model = BicycleModel(wheelbase)
        self.vehiclePose = np.array(vehiclePose)

    def getRanges(self):
        '''
        Returns range from vehicle to each base station as 1D np array
        '''
        return np.linalg.norm(self.baseStationLocations - self.vehiclePose[:2], axis=1)

    def getRssiMeasurements(self):
        ranges = self.getRanges()
        print("ranges: %s" %ranges)
        measurements = self.baseStationPowerTxs - 10 * self.rssiN * np.log10(ranges)
        print("expected measurements: %s" %measurements)
        measurements += np.random.normal(0, self.rssiStdDev, size=measurements.size)
        print("simulated measurements: %s" %measurements)
        return measurements

    def getTotMeasurements(self):
        '''
        TODO: Add noise
        '''
        ranges = self.getRanges()
        return ranges / self.speedSound
    
    def getRangesFromRssi(self):
        rssiMeasurements = self.getRssiMeasurements()
        ranges = 10 ** ((self.baseStationPowerTxs - rssiMeasurements) / (10 * self.rssiN))
        return ranges

    def getRangesFromTot(self):
        totMeasurements = self.getTotMeasurements()
        ranges = totMeasurements * self.speedSound
        return ranges

    def lateration(self):
        pass

    def move(self, u, timestep=0.1):
        '''
        Move vehicle given control input and timestep
        '''
        self.vehiclePose = self.model.stepKinematic(self.vehiclePose, u, timestep=timestep)

class Test(unittest.TestCase):
    def testGetRanges(self):
        baseStationLocations = [(1, 1), (-1, -1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs)
        ranges = sim.getRanges()
        print(ranges)

    def testRssi(self):
        baseStationLocations = [(1, 1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs, rssiStdDev=0)
        rangesActual = sim.getRanges()
        rangesRssi = sim.getRangesFromRssi()
        self.assertEqual(rangesActual, rangesRssi)

    def testTot(self):
        baseStationLocations = [(1, 1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs)
        rangesActual = sim.getRanges()
        rangesTot = sim.getRangesFromTot()
        self.assertEqual(rangesActual, rangesTot)

    def testMove(self):
        baseStationLocations = [(1, 1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs)
        u = np.array([1, 10 * np.pi / 180])
        for i in range(1, 1001):
            sim.move(u)
            if i % 100 == 0:
                print('vehicle pose: %s' %sim.vehiclePose)

if __name__ == "__main__":
    print("Simulation")