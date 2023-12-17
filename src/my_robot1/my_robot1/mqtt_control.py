import struct
import paho.mqtt.client as mqtt


class MqttControl:
    CONTROL_TOPIC = 'tank/in'
    TELEMETRY_TOPIC = 'tank/out'
    CENTER = 100

    COMMAND_START = 102
    COMMAND_STOP = 101
    COMMAND_START_STREAM = 120

    def __init__(self, host, port):

        self.host = host
        self.port = port
        self.client = mqtt.Client()
        self.client.connect(host, port)
        print("Connected to MQTT broker at {}:{}".format(host, port))

    def sendControl(self, left_right, forward_backward, command):
        channels = [left_right, forward_backward, command, self.CENTER]
        start_byte = '$'
        num_bytes = len(channels)
        message = start_byte.encode('utf-8')
        message += struct.pack('B', num_bytes)
        for channel in channels:
            message += struct.pack('B', channel)

        self.client.publish(self.CONTROL_TOPIC, message)

    def enable_roomba(self):
        self.sendControl(self.CENTER, self.CENTER, self.COMMAND_START)
        self.sendControl(self.CENTER, self.CENTER, self.COMMAND_START_STREAM)

    def disable_roomba(self):
        self.sendControl(self.CENTER, self.CENTER, self.COMMAND_STOP)

    def on_message(self, client, userdata, message):
        pass
        # print("Received message: ", str(message.payload.decode("utf-8")))

        # self.sendControl(self.CENTER, self.CENTER)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        self.client.subscribe(self.TELEMETRY_TOPIC)

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected with result code " + str(rc))

    def run(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.loop_start()


if __name__ == '__main__':
    MQTT_BROKER_HOST = '192.168.0.19'
    mqtt = MqttControl(MQTT_BROKER_HOST, 1883)
    mqtt.run()
