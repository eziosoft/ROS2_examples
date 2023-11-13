import socket
import time


class LidarOutput:
    def __init__(self):
        self.timeStamp = 0
        self.speed = 0
        self.angOffset = 0
        self.angStart = 0
        self.nSamples = 0
        self.distances = []
        self.angles = []
        self.error = False
        self.errorMsg = ""


class MyLidar:

    def __init__(self):
        self.IP = 'Lidar'
        self.port=23
        self.socket = None
        self.data_available_callback = None

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            host_ip = socket.gethostbyname(self.IP)
            self.socket.connect((host_ip, self.port))
            self.socket.settimeout(1)
            return True
        except:
            return False
        
    def disconnect(self):
        self.socket.close()

    def start_parse(self):
        while True:
            try:
                data = self.socket.recv(1024)
                if self.data_available_callback is not None:
                    self.data_available_callback(data)
            except:
                pass


    def checksum(self, frame):
        cs = int.from_bytes(frame[-2:], byteorder='big', signed=False)
        result = 0

        for i in frame[0:-2]:
            result += int(i)

        return result == cs

    def parseData(self, payload, payloadLen):
        out = LidarOutput()

        speed = payload[0]
        out.speed = speed * 0.05  # r/s
        # print ('RPM: %.2fr/s or %drpm'%(speed, speed*60))

        angOffset = int.from_bytes(payload[1:3], byteorder='big', signed=True)
        out.angOffset = angOffset * 0.01
        # print ('Angle Offset: %.2f'%angOffset)

        angStart = int.from_bytes(payload[3:5], byteorder='big', signed=False)
        out.angStart = angStart * 0.01
        # print ('Starting Angle: %.2f'%angStart)

        out.nSamples = int((payloadLen - 5) / 3)
        # print("N Samples: %d"%nSamples)

        for i in range(out.nSamples):
            index = 5 + i * 3
            sampleID = payload[index]
            distance = int.from_bytes(
                payload[index + 1:index + 3], byteorder='big', signed=False)
            out.distances.append(distance)
            ang = angStart + 22.5 * i / out.nSamples
            out.angles.append(ang)
            print('%.2f: %.2f mm' % (ang, distance))

        out.timeStamp = time.time()
        out.error = False
        return out

    def parseError(self, payload):
        out = LidarOutput()
        speed = payload[0]
        out.speed = speed * 0.05  # r/s
        print('Error: Low RPM - %.2fr/s or %drpm' % (speed, speed * 60))
        out.error = True
        out.errorMsg = 'Low RPM - %.2fr/s or %drpm' % (speed, speed * 60)
        return out

    def processFrame(self, frame):
        out: LidarOutput = None

        if len(frame) < 3:
            return False
        frameLen = int.from_bytes(frame[1:3], byteorder='big', signed=False)
        if len(frame) < frameLen + 2:
            return False  # include 2bytes checksum

        if not self.checksum(frame):
            return True  # checksum failed

        try:
            protocalVer = frame[3]  # 0x00
            frameType = frame[4]  # 0x61
            payloadType = frame[5]  # 0xAE or 0XAD
            payloadLen = int.from_bytes(
                frame[6:8], byteorder='big', signed=False)

            if payloadType == 0xAD:
                out = self.parseData(frame[8:frameLen + 1], payloadLen)
            elif payloadType == 0xAE:
                out = self.parseError(frame[8:frameLen + 1])
        except:
            pass
        return out


def data_available_callback(data):
    print(data)

if __name__ == '__main__':
    lidar = MyLidar()
    lidar.data_available_callback = data_available_callback

    lidar.connect()
    lidar.start_parse()