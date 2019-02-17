#!/usr/bin/env python3

from networktables import NetworkTables

import time


class Network:
    def __init__(self, nt_ip):
        NetworkTables.initialize(server=nt_ip)
        self.nt = NetworkTables.getTable("NetworkUsage")

    def write(self, msg):
        self.nt.putNumber("bandwidth", msg)

class UsageParse:
    ETH0_ROW = 2
    TRANSMIT_COL = 9
    RECEIVE_COL = 1

    @staticmethod
    def read_file(path="/proc/net/dev"):
        with open(path, 'r') as f:
            data = [line.split() for line in f.readlines()]
            return int(data[UsageParser.ETH0_ROW][UsageParser.TRANSMIT_COL]) + int(data[UsageParser.ETH0_ROW][UsageParser.RECEIVE_COL])

if __name__ == '__main__':
    TIME_DELAY = 3
    UNIT_CONVERSION = 10 ** 6

    net = Network('roborio-263-FRC.local')
    previous_usage = 0

    while True:
        total_usage = UsageParse.read_file()
        net.write(float(total_usage - previous_usage) / (TIME_DELAY * UNIT_CONVERSION))
        previous_usage = total_usage

        time.sleep(TIME_DELAY)

