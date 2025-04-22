#!/usr/bin/env python3

import numpy as np

import unittest

class RadioModel:
    def __init__(self, baseStationLocations, baseStationAs, rssiN=2, rssiStdDev=0.5,
            totStdDev=1e-3, vehiclePose=(0, 0, 0)):
        '''
        baseStationLocations is a list of tuples of the form (x, y) specifying location in meters
        baseStationAs is a list of RSSI at 1 meter
        vehiclePose is a tuple of (x, y, yaw) in meters and radians
        '''
        self.SPEED_OF_LIGHT = 299792458.0 # m/s
        self.baseStationLocations = np.array(baseStationLocations)
        self.baseStationAs = np.array(baseStationAs)
        self.rssiN = rssiN
        self.rssiStdDev = rssiStdDev
        self.totStdDev = totStdDev
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
        measurements = self.baseStationAs - 10 * self.rssiN * np.log10(ranges)
        print("expected measurements: %s" %measurements)
        measurements += np.random.normal(0, self.rssiStdDev, size=measurements.size)
        print("simulated measurements: %s" %measurements)
        return measurements

    def getTotMeasurements(self):
        '''
        TODO: Add noise
        '''
        ranges = self.getRanges()
        return ranges / self.SPEED_OF_LIGHT
    
    def getRangesFromRssi(self):
        rssiMeasurements = self.getRssiMeasurements()
        ranges = 10 ** ((self.baseStationAs - rssiMeasurements) / (10 * self.rssiN))
        return ranges

    def getRangesFromTot(self):
        totMeasurements = self.getTotMeasurements()
        ranges = totMeasurements * self.SPEED_OF_LIGHT
        return ranges

    def lateration(self):
        pass

class Test(unittest.TestCase):
    def testGetRanges(self):
        baseStationLocations = [(1, 1), (-1, -1)]
        baseStationAs = [-20] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs)
        ranges = rm.getRanges()
        print(ranges)

    def testRssi(self):
        baseStationLocations = [(1, 1)]
        baseStationAs = [-20] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs, rssiStdDev=0)
        rangesActual = rm.getRanges()
        rangesRssi = rm.getRangesFromRssi()
        self.assertEqual(rangesActual, rangesRssi)

    def testTot(self):
        baseStationLocations = [(1, 1)]
        baseStationAs = [-20] * len(baseStationLocations)
        rm = RadioModel(baseStationLocations, baseStationAs)
        rangesActual = rm.getRanges()
        rangesTot = rm.getRangesFromTot()
        self.assertEqual(rangesActual, rangesTot)
