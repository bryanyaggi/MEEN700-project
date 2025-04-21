#!/usr/bin/env python3

from collections import deque

import unittest

class MovingAverager:
    def __init__(self, buffer=10):
        self.d = deque(maxlen=buffer)

    def append(self, value):
        self.d.append(value)

    def getAverage(self):
        if len(self.d) == 0:
            return 0
        return sum(self.d) / len(self.d)

class Test(unittest.TestCase):
    def testBuffer(self):
        ma = MovingAverager()
        for val in range(20):
            ma.append(val)
            print(ma.d)

    def testAverage(self):
        ma = MovingAverager(buffer=5)
        print(ma.getAverage())
        for val in range(20):
            ma.append(val)
            print(ma.getAverage())
