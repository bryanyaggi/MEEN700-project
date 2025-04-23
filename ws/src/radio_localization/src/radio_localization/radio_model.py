#!/usr/bin/env python3

import numpy as np

import unittest
        
SPEED_OF_LIGHT = 299792458.0 # m/s
SECONDS_PER_100NS = 1e-7

class RadioModel:
    def __init__(self, baseStationLocations, baseStationAs, baseStationNs, rssiStdDev=0.5,
            tofStdDev=1e-3, vehiclePose=(0, 0, 0)):
        '''
        baseStationLocations is a list of tuples of the form (x, y) specifying location in meters
        baseStationAs is a list of RSSI at 1 meter
        vehiclePose is a tuple of (x, y, yaw) in meters and radians
        '''
        self.baseStationLocations = np.array(baseStationLocations)
        self.baseStationAs = np.array(baseStationAs)
        self.baseStationNs =np.array(baseStationNs)
        self.rssiStdDev = rssiStdDev
        self.tofStdDev = tofStdDev
        self.vehiclePose = np.array(vehiclePose)

    def getRanges(self):
        '''
        Returns range from vehicle to each base station as 1D np array
        '''
        return np.linalg.norm(self.baseStationLocations - self.vehiclePose[:2], axis=1)

    def getRssiMeasurements(self):
        '''
        Simulate measurements with noise.
        '''
        ranges = self.getRanges()
        print("ranges: %s" %ranges)
        measurements = self.baseStationAs - 10 * self.baseStationNs * np.log10(ranges)
        print("expected measurements: %s" %measurements)
        measurements += np.random.normal(0, self.rssiStdDev, size=measurements.size)
        print("simulated measurements: %s" %measurements)
        return measurements

    def getTofMeasurements(self):
        '''
        Simulate measurements. TODO: Add noise.
        '''
        ranges = self.getRanges()
        return ranges / SPEED_OF_LIGHT
    
    def getRangesFromRssi(self, rssiMeasurements):
        ranges = 10 ** ((self.baseStationAs - rssiMeasurements) / (10 * self.baseStationNs))
        return ranges

    def getRangesFromTof(self, tofMeasurements):
        ranges = tofMeasurements * SECONDS_PER_100NS * SPEED_OF_LIGHT
        return ranges

    def lateration(self):
        pass

class Test(unittest.TestCase):
    def testGetRanges(self):
        baseStationLocations = [(1, 1), (-1, -1)]
        baseStationAs = [-20] * len(baseStationLocations)
        baseStationNs = [2] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs, baseStationNs)
        ranges = rm.getRanges()
        print(ranges)

    def testRssi(self):
        baseStationLocations = [(1, 1)]
        baseStationAs = [-20] * len(baseStationLocations)
        baseStationNs = [2] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs, baseStationNs, rssiStdDev=0)
        rangesActual = rm.getRanges()
        simRssiMeasurements = rm.getRssiMeasurements()
        rangesRssi = rm.getRangesFromRssi(simRssiMeasurements)
        self.assertEqual(rangesActual, rangesRssi)

    def testTof(self):
        baseStationLocations = [(1, 1)]
        baseStationAs = [-20] * len(baseStationLocations)
        baseStationNs = [2] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs, baseStationNs, tofStdDev=0)
        rangesActual = rm.getRanges()
        simTofMeasurements = rm.getTofMeasurements()
        rangesTof = rm.getRangesFromTot(simTofMeasurements)
        self.assertEqual(rangesActual, rangesTof)
