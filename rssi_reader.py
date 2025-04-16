#!/usr/bin/env python3

import socket

class RssiReader:
    def __init__(self, ip="192.168.0.199", port=30000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))
        print("Listening for RSSI packets on %s:%s..." %(ip, port))

    def run(self):
        while True:
            data, _ = self.socket.recvfrom(1500) # Silvus max packet size is 1400
            print("Received RSSI report.")

    def __del__(self):
        self.socket.close()

if __name__ == "__main__":
    rr = RssiReader()
    rr.run()