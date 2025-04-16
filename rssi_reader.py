#!/usr/bin/env python3

import socket
import struct

def parsePacket(packet):
    '''
    Parses data stored in type-length-value (TLV) format
    '''
    offset = 0
    report = {}

    while offset + 4 <= len(packet):
        # Unpack type and length
        type_, length = struct.unpack_from('!HH', packet, offset)
        # ! - big-endian order
        # H - unsigned short
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
            print(value)

class RssiReader:
    def __init__(self, ip="192.168.0.199", port=30000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))
        print("Listening for RSSI packets on %s:%s..." %(ip, port))

    def run(self):
        while True:
            packet, _ = self.socket.recvfrom(1500) # Silvus max packet size is 1400
            #print("Received RSSI report.")
            parsePacket(packet)

    def __del__(self):
        self.socket.close()

if __name__ == "__main__":
    rr = RssiReader()
    rr.run()