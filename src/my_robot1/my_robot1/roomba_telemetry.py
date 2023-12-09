import struct
import time
from my_robot1.create2_enums import *
import paho.mqtt.client as mqtt


class Create2Driver:
    def __init__(self):
        self._buffer = bytes()

    def update(self, data):
        self._buffer += data
        i = 0
        state = None
        while i < len(self._buffer) - 1:
            if self._buffer[i] == 19:
                size = self._buffer[i + 1]
                # print("found 19 " + str(size))
                if len(self._buffer) > size + i + 2:
                    # print("long enough")
                    # check checksum
                    checksum = 0
                    for j in range(i, size + i + 3):
                        checksum += self._buffer[j]

                    if (checksum & 0xFF) == 0:
                        # parse packet
                        pos = i + 2
                        state = State()
                        while pos < i + size + 2:
                            sensor = self._buffer[pos]
                            pos += 1
                            if sensor == Sensor.OIMode:
                                fmt = ">B"
                                (state.mode,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.ChargingState:
                                fmt = ">B"
                                (state.chargingState,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                state.chargingState = ChargingState(state.chargingState)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Voltage:
                                fmt = ">H"
                                (state.voltageInMV,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Current:
                                fmt = ">h"
                                (state.currentInMA,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Temperature:
                                fmt = ">B"
                                (state.temperatureInDegCelcius,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.BatteryCharge:
                                fmt = ">H"
                                (state.batteryChargeInMAH,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.BatteryCapacity:
                                fmt = ">H"
                                (state.batteryCapacityInMAH,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffLeftSignal:
                                fmt = ">H"
                                (state.cliffLeftSignalStrength,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffFrontLeftSignal:
                                fmt = ">H"
                                (
                                    state.cliffFrontLeftSignalStrength,
                                ) = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffFrontRightSignal:
                                fmt = ">H"
                                (
                                    state.cliffFrontRightSignalStrength,
                                ) = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffRightSignal:
                                fmt = ">H"
                                (state.cliffRightSignalStrength,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LeftEncoderCounts:
                                fmt = ">h"
                                (state.leftEncoderCounts,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.RightEncoderCounts:
                                fmt = ">h"
                                (state.rightEncoderCounts,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterOmni:
                                fmt = ">B"
                                (state.infraredCharacterOmni,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterLeft:
                                fmt = ">B"
                                (state.infraredCharacterLeft,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterRight:
                                fmt = ">B"
                                (state.infraredCharacterRight,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpLeftSignal:
                                fmt = ">H"
                                (state.lightBumpLeftSignal,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpFrontLeftSignal:
                                fmt = ">H"
                                (state.lightBumpFrontLeftSignal,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpCenterLeftSignal:
                                fmt = ">H"
                                (state.lightBumpCenterLeftSignal,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpCenterRightSignal:
                                fmt = ">H"
                                (
                                    state.lightBumpCenterRightSignal,
                                ) = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpFrontRightSignal:
                                fmt = ">H"
                                (state.lightBumpFrontRightSignal,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpRightSignal:
                                fmt = ">H"
                                (state.lightBumpRightSignal,) = struct.unpack_from(
                                    fmt, self._buffer, pos
                                )
                                pos += struct.calcsize(fmt)
                            else:
                                pass
                                # print(str.format("Don't know {}", sensor))

                        # return state
                    else:
                        self._buffer = bytes()
                        print("Checksum incorrect!")

                    # delete parsed portion of buffer
                    self._buffer = self._buffer[size + i :]
                    i = -1
            i += 1

        return state


class OdonometryDriver:
    DATA_TOPIC = "tank/stream"

    def __init__(self, broker: str, port: int, on_state_callback=None) -> None:
        self.on_state_callback = on_state_callback

        self.create2_driver = Create2Driver()
        self.state = None
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(broker, port, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.DATA_TOPIC)

    def on_message(self, client, userdata, msg):
        # print(msg.topic+" "+str(msg.payload))
        self.update(msg.payload)

    def update(self, data):
        new_state = self.create2_driver.update(data)
        if new_state is not None:
            self.state = new_state
            if self.on_state_callback is not None:
                self.on_state_callback(self.state)

    def get_state(self):
        return self.state


if __name__ == "__main__":
    odometry = OdonometryDriver("192.168.0.102", 1883)
    left = 0
    right = 0
    
    while True:
        state = odometry.get_state()
        if (
            state is not None
            and hasattr(state, "leftEncoderCounts")
            and hasattr(state, "rightEncoderCounts")
        ):
            left = state.leftEncoderCounts
            right = state.rightEncoderCounts
            print(left, right)
        time.sleep(0.1)
