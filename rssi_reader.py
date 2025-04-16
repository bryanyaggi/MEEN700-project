#!/usr/bin/env python3

import numpy as np
import socket
import struct

rssiAntennaTypes = [5000, 5001, 5002, 5003]

def parsePacket(packet):
    '''
    Parses data stored in type-length-value (TLV) format
    '''
    offset = 0
    nodeId = None
    rssi = np.zeros(4)

    while offset + 4 <= len(packet):
        # Unpack type and length
        type_, length = struct.unpack_from('!HH', packet, offset)
        # ! - big-endian order
        # H - unsigned short (2 bytes)
        offset += 4

        # Unpack value
        valueRaw = packet[offset:offset + length]
        offset += length

        # Decode value
        try:
            value = valueRaw.decode('ascii').strip('\x00').strip()
        except UnicodeDecodeError:
            value = ''

        if type_ == 5007:
            nodeId = value
        else:
            try:
                i = rssiAntennaTypes.index(type_)
            except ValueError:
                continue
            rssi[i] = value

    return nodeId, rssi

class RssiReader:
    def __init__(self, ip="192.168.132.15", port=30000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))
        print("Listening for RSSI packets on %s:%s..." %(ip, port))

    def run(self):
        while True:
            packet, _ = self.socket.recvfrom(1500) # Silvus max packet size is 1400
            #print("Received RSSI report.")
            nodeId, rssi = parsePacket(packet)
            if nodeId != '0':
                print(nodeId, rssi)

    def __del__(self):
        self.socket.close()

if __name__ == "__main__":
    rr = RssiReader()
    rr.run()