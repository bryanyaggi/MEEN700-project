#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

import unittest

class Simulation:
    def __init__(self, baseStationLocations, baseStationPowerTxs, rssiN=2, rssiStdDev=1, speedSound=343,
                 totStdDev=1e-3, vehiclePose=(0, 0, 0)):
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
        self.vehiclePose = np.array(vehiclePose)

    def getDistances(self):
        '''
        Returns distance from vehicle to each base station as 1D np array
        '''
        return np.linalg.norm(self.baseStationLocations - self.vehiclePose[:2], axis=1)

    def getRssiMeasurements(self):
        distances = self.getDistances()
        print("distances: %s" %distances)
        measurements = self.baseStationPowerTxs - 10 * self.rssiN * np.log10(distances)
        print("expected measurements: %s" %measurements)
        measurements += np.random.normal(0, self.rssiStdDev, size=measurements.size)
        print("simulated measurements: %s" %measurements)
        return measurements

    def getTotMeasurements(self):
        '''
        TODO: Add noise
        '''
        distances = self.getDistances()
        return distances / self.speedSound
    
    def getDistancesFromRssi(self):
        rssiMeasurements = self.getRssiMeasurements()
        distances = 10 ** ((self.baseStationPowerTxs - rssiMeasurements) / (10 * self.rssiN))
        return distances

    def getDistancesFromTot(self):
        totMeasurements = self.getTotMeasurements()
        distances = totMeasurements * self.speedSound
        return distances

    def lateration(self):
        pass

    def plotInit(self):
        pass

    def plot(self):
        fig, ax = plt.subplots()

        # vehicle pose
        l = 1e-4
        dx = l * np.cos(self.vehiclePose[2])
        dy = l * np.sin(self.vehiclePose[2])
        ax.scatter(self.vehiclePose[0], self.vehiclePose[1], color='black', label='vehicle pose')
        ax.quiver(self.vehiclePose[0], self.vehiclePose[1], dx, dy, units='inches', angles='xy')

        # base stations
        ax.scatter(self.baseStationLocations[:, 0], self.baseStationLocations[:, 1], color='blue', label='base stations')
        distances = self.getDistancesFromRssi()
        for i in range(len(distances)):
            circle = patches.Circle((self.baseStationLocations[i][0], self.baseStationLocations[i][1]), distances[i],
                                    edgecolor='blue', fill=False)
            ax.add_patch(circle)
        
        ax.set_xlabel('x [meters]')
        ax.set_ylabel('y [meters]')
        #ax.legend()
        ax.set_aspect('equal')
        plt.show()

class Test(unittest.TestCase):
    def testGetDistances(self):
        baseStationLocations = [(1, 1), (-1, -1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs)
        dists = sim.getDistances()
        print(dists)

    def testRssi(self):
        baseStationLocations = [(1, 1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs, rssiStdDev=0)
        distancesActual = sim.getDistances()
        distancesRssi = sim.getDistancesFromRssi()
        self.assertEqual(distancesActual, distancesRssi)

    def testTot(self):
        baseStationLocations = [(1, 1)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        sim = Simulation(baseStationLocations, baseStationPowerTxs)
        distancesActual = sim.getDistances()
        distancesTot = sim.getDistancesFromTot()
        self.assertEqual(distancesActual, distancesTot)

    def testPlot(self):
        baseStationLocations = [(1000, 1000), (-1000, -1000), (-1000, 1000)]
        baseStationPowerTxs = [20] * len(baseStationLocations)
        vehiclePose = (0, 0, np.pi/4)
        sim = Simulation(baseStationLocations, baseStationPowerTxs, vehiclePose=vehiclePose)
        sim.plot()

if __name__ == "__main__":
    print("Simulation")