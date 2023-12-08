import socket
import sys
import time


class LidarOutput:
    def __init__(self):
        self.timeStamp = 0
        self.speed_rev_s = 0
        self.angOffset = 0
        self.angStart = 0
        self.nSamples = 0
        self.distances = []
        self.angles = []
        self.sampleId = []
        self.error = False
        self.errorMsg = ""


class LidarClient:
    def __init__(self, ip, port, callback=None):
        self.IP = ip
        self.port = port
        self.last_angle = 0
        self.frame = ''
        self.socket = None
        self.callback = callback

    def create_socket(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Socket successfully created")
        except socket.error as err:
            print("Socket creation failed with error %s" % (err))
            sys.exit()

    def resolve_host(self):
        try:
            host_ip = socket.gethostbyname(self.IP)
        except socket.gaierror:
            print("There was an error resolving the host")
            exit()
        return host_ip

    def connect_to_server(self):
        self.create_socket()
        host_ip = self.resolve_host()
        print("Connecting to %s (%s) on port %s" %
              (self.IP, host_ip, self.port))
        self.socket.connect((host_ip, self.port))
        print("The socket has successfully connected")

    def out(self, lidarOutput: LidarOutput):
        if self.callback is not None:
            self.callback(lidarOutput)

    def checksum(self, frame):
        cs = int.from_bytes(frame[-2:], byteorder='big', signed=False)
        result = sum(frame[:-2])
        return result == cs

    def parse_data(self, payload, payload_len):
        measurements = []
        angles = []
        sample_ids = []

        speed_r_s = payload[0] * 0.05
        ang_offset = int.from_bytes(
            payload[1:3], byteorder='big', signed=True) * 0.01
        ang_start = int.from_bytes(
            payload[3:5], byteorder='big', signed=False) * 0.01
        n_samples = int((payload_len - 5) / 3)

        # print('start: %.2f, offset: %.2f, samples: %d,  speed: %.2fr/s or %drpm' % (
        #     ang_start, ang_offset, n_samples, speed, speed * 60))

        for i in range(n_samples):
            index = 5 + i * 3
            sample_id = payload[index]
            ang = ang_start + 22.5 * i / n_samples
            dist = int.from_bytes(
                payload[index + 1:index + 3], byteorder='big', signed=False) * 0.25

            measurements.append(dist/1000.0)
            angles.append(ang)
            sample_ids.append(sample_id)

        lidarOut = LidarOutput()
        lidarOut.timeStamp = time.time()
        lidarOut.speed_rev_s = speed_r_s
        lidarOut.angOffset = self.calculate_average_distance_between_elenents(
            angles)
        lidarOut.angStart = ang_start
        lidarOut.nSamples = n_samples
        lidarOut.distances = measurements
        lidarOut.angles = angles
        lidarOut.sampleId = sample_ids
        lidarOut.error = False
        lidarOut.errorMsg = ""

        self.out(lidarOut)
        measurements.clear()
        angles.clear()

    def parse_error(self, payload):
        speed = payload[0] * 0.05
        # print('Error: Low RPM - %.2fr/s or %drpm' % (speed, speed * 60))
        lidar_out = LidarOutput()
        lidar_out.error = True
        lidar_out.errorMsg = 'Error: Low RPM - %.2fr/s or %drpm' % (
            speed, speed * 60)
        self.out(lidar_out)

    def process_frame(self, frame):
        if len(frame) < 3:
            return False
        frame_len = int.from_bytes(frame[1:3], byteorder='big', signed=False)
        if len(frame) < frame_len + 2:
            return False

        if not self.checksum(frame):
            return True

        try:
            protocol_ver = frame[3]
            frame_type = frame[4]
            payload_type = frame[5]
            payload_len = int.from_bytes(
                frame[6:8], byteorder='big', signed=False)

            if payload_type == 0xAD:
                self.parse_data(frame[8:frame_len + 1], payload_len)
            elif payload_type == 0xAE:
                self.parse_error(frame[8:frame_len + 1])
        except Exception as e:
            print(f"Error processing frame: {e}")
        return True

    def on_data(self, data):
        if data == b'\xaa' and len(self.frame) == 0:
            self.frame = data
        elif len(self.frame) > 0:
            self.frame += data
            if self.process_frame(self.frame):
                self.frame = ''

    def start_listening(self):
        while True:
            try:
                self.on_data(self.socket.recv(1))
            except (socket.error, ConnectionResetError):
                print("Connection lost. Reconnecting...")
                lidarOut = LidarOutput()
                lidarOut.error = True
                lidarOut.errorMsg = "Connection lost. Reconnecting..."
                self.out(lidarOut)

                self.socket.close()
                time.sleep(2)  # You can adjust the sleep duration as needed
                self.connect_to_server()

    def calculate_average_distance_between_elenents(self, arr):
        sum = 0
        for i in range(1, len(arr)):
            sum += arr[i] - arr[i - 1]
        return sum / (len(arr) - 1)


def lidar_callback(lidarOutput: LidarOutput):
    print(f"Time: {lidarOutput.timeStamp}")
    print(f"Speed: {lidarOutput.speed_rev_s}")
    print(f"Angle Offset: {lidarOutput.angOffset}")
    print(f"Angle Start: {lidarOutput.angStart}")
    print(f"Number of Samples: {lidarOutput.nSamples}")
    print(f"Distances: {lidarOutput.distances}")
    print(f"Angles: {lidarOutput.angles}")
    print(f"Sample IDs: {lidarOutput.sampleId}")
    print(f"Error: {lidarOutput.error}")
    print(f"Error Message: {lidarOutput.errorMsg}")
    print("")


if __name__ == "__main__":
    # IP = '192.168.8.200'
    IP = '127.0.0.1'
    # PORT = 23
    PORT = 2323

    print("Starting Lidar Client...")

    lidar_client = LidarClient(IP, PORT, callback=lidar_callback)

    lidar_client.connect_to_server()
    lidar_client.start_listening()
