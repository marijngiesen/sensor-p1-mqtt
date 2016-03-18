#!/usr/bin/env python3

import re
import sys
import serial
import paho.mqtt.client as mqtt


class P1Sensor(object):
    def __init__(self):
        self.initialized = False
        self.serial = serial.Serial()

    def init_connection(self):
        self.serial.baudrate = 9600
        self.serial.bytesize = serial.SEVENBITS
        self.serial.parity = serial.PARITY_EVEN
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.xonxoff = 1
        self.serial.rtscts = 0
        self.serial.timeout = 20
        self.serial.port = "/dev/ttyUSB0"

        try:
            self.serial.open()
        except:
            print("Error opening serial connection")
            sys.exit(1)

        self.initialized = True

    def read_data(self):
        if not self.initialized:
            self.init_connection()

        try:
            raw_data = self.serial.readline()
        except KeyboardInterrupt:
            self.serial.close()
            sys.exit(0)
        except:
            self.serial.close()
            print("Error reading data from serial connection")
            sys.exit(1)

        return str(raw_data, "utf-8").strip()


if __name__ == "__main__":
    p1sensor = P1Sensor()

    mqttc = mqtt.Client()
    mqttc.connect("192.168.1.4")
    mqttc.loop_start()

    previous_power = 0
    previous_gas = 0

    while True:
        data = p1sensor.read_data()
        power = re.match(r"1\-0:1\.7\.0\(([0-9\.]+)\*kW\)", data)
        if power is not None:
            current_power = float(power.group(1))

            if current_power != previous_power:
                mqttc.publish("home/energy/power", "{:0.3f}".format(current_power))
                previous_power = current_power

        gas = re.match(r"\(([0-9\.]+)\)", data)
        if gas is not None:
            current_gas = float(gas.group(1))

            if current_gas != previous_gas:
                publish_value = current_gas - previous_gas if previous_gas > 0 else 0
                mqttc.publish("home/energy/gas", "{:0.3f}".format(publish_value))
                previous_gas = current_gas
