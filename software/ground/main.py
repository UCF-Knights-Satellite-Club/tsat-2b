import struct
import enum
from dataclasses import dataclass
import typing
import serial
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation
import threading

class PacketType(enum.Enum):
    PING = 1
    TELEMETRY = 2

@dataclass
class PingPacket:
    counter: int

# Ideally this would be a 1 to 1 representation
# of the C code, but to make things simpler the
# fixed-point numbers are made floating during deserialization
@dataclass
class TelemetryPacket:
    frame_count: int
    time: float
    altitude: float
    pressure: float
    temperature: float
    acceleration_magnitude: float
    velocity: float

@dataclass
class PacketMeta:
    satellite_id: int
    packet_type: PacketType

@dataclass
class Packet:
    meta: PacketMeta
    data: TelemetryPacket | PingPacket

@dataclass
class PlotInfo:
    fancy_name: str
    label: str
    data_name: str
    row: int
    col: int

telemetry_data: list[TelemetryPacket] = []

def deserialize_packet(raw: bytes) -> typing.Optional[Packet]:
    # see https://docs.python.org/3/library/struct.html
    # esp32 is little endian
    # <2B = little endian, 2 bytes
    id, ty = struct.unpack("<2H", raw[:4])
    try:
        ty = PacketType(ty)
    except ValueError:
        print(f"Unknown packet: {ty}")
        return None
    
    meta = PacketMeta(id, ty)
    data = None
    match meta.packet_type:
        case PacketType.PING:
            # <I = little endian, 1 32bit uint
            data = PingPacket(struct.unpack("<I", raw[4:]))
        case PacketType.TELEMETRY:
            # <7I = little endian, 7 32bit uints
            raw = list(struct.unpack("<7I", raw[4:]))
            # 1-6 are fixed-point floats with 3 decimals
            # Keep in mind range ends are exclusive!!
            for i in range(1, 7):
                raw[i] /= 1000

            data = TelemetryPacket(*raw)
    
    return Packet(meta, data)

def serialize_packet(packet: Packet) -> bytes:
    # <2B = little endian, 2 bytes
    b = struct.pack("<2B", packet.meta.satellite_id, packet.meta.packet_type)

    match packet.meta.packet_type:
        case PacketType.PING:
            b += struct.pack("<I", packet.data.counter)
        # We never send telemetry packets so we dont need the code for it

    return b

fig, axs = plt.subplots(2, 3)
plot_info = [
    PlotInfo("Temperature", "°C", "temperature", 0, 0),
    PlotInfo("Pressure", "pa", "pressure", 0, 1),
    PlotInfo("Altitude", "m", "altitude", 0, 2),
    PlotInfo("Acceleration", "m/s/s", "acceleration_magnitude", 1, 0),
    PlotInfo("Ascent Velocity", "m/s", "velocity", 1, 1)
]
line_data = []
lines = []
for (i, plot) in enumerate(plot_info):
    ax = axs[plot.row, plot.col]
    line = ax.plot([], [])
    ax.grid()
    ax.set_xlim(0, 1)
    ax.set_ylim(-100, 100)
    ax.set_title(plot.fancy_name)
    ax.set_ylabel(plot.label)
    line_data.append([[], []])
    lines.append(line)

def run(data):
    tl, yl = data
    if len(tl) == 0:
        return [line[0] for line in lines]
    
    for (i, plot) in enumerate(plot_info):
        for (t, y) in zip(tl, yl):
            line_data[i][0].append(t)
            line_data[i][1].append(getattr(y, plot.data_name))

            ax = axs[plot.row, plot.col]

            xmin, xmax = ax.get_xlim()

            if t >= xmax:
                ax.set_xlim(t-100, t)

        lines[i][0].set_data(line_data[i][0], line_data[i][1])

    return [line[0] for line in lines]

def data_gen():
    i = 0
    while True:
        tl = []
        yl = []
        while len(telemetry_data) > i:
            packet = telemetry_data[i]
            i += 1
            tl.append(packet.frame_count)
            yl.append(packet)
        yield tl, yl

# Need blit=True otherwise performance is very bad
# We use one FuncAnimation to increase performance further
anim = animation.FuncAnimation(fig, run, data_gen, save_count=100, blit=True, interval=1/60)

img_ax = axs[1, 2]
with open("ksc.png", "rb") as f:
    image = plt.imread(f)
img_ax.imshow(image)
img_ax.axis("off")

parser = argparse.ArgumentParser(
    description = "TSAT-2B Communications"
)
parser.add_argument("reciever", help="The serial port in which the reciever is connected to the computer.\nUsually COM1 on Windows and /dev/ttyUSB0 or /dev/ttyS0 on Linux/Mac")
args = parser.parse_args()
ser = serial.Serial(args.reciever, baudrate=115200)

running = True
def run():
    #i=0
    while running:
        #telemetry_data.append(TelemetryPacket(
        #    i,
        #    i * 0.1,
        #    0,
        #    i * 10,
        #    40*np.sin(i),
        #    0,
        #    0
        #))
        #i += 1
        # Simple code to read every packet.
        # Eventually we need to do
        # reading and writing simultaneously
        raw = ser.readline()
        try:
            raw = [int(i) for i in raw.decode().strip().split(' ')]
            print("Raw:", raw)
            line = bytes(raw)
            packet = deserialize_packet(line)
            print("Packet:", packet)
            if packet.meta.packet_type == PacketType.TELEMETRY:
                telemetry_data.append(packet.data)
            print()
        except:
            continue

t = threading.Thread(target=run)
t.start()

plt.show(block=True)
running = False
t.join()