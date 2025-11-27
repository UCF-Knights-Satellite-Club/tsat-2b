import struct
import enum
from dataclasses import dataclass
import typing
import serial
import argparse
import numpy as np
import time
import threading
from dash import Dash, Output, html, dcc, Input, State, callback, set_props
import dash_bootstrap_components as dbc
import random

# --------------------[ Packets ]--------------------

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
            # <6Ii = little endian, 6 32bit uints, 1 32bit int
            raw = list(struct.unpack("<6Ii", raw[4:]))
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

# --------------------[ Plotting ]--------------------

@dataclass
class PlotInfo:
    fancy_name: str
    label: str
    data_name: str
    row: int
    col: int

plot_info = [
    PlotInfo("Temperature", "°C", "temperature", 1, 1),
    PlotInfo("Pressure", "pa", "pressure", 1, 2),
    PlotInfo("Altitude", "m", "altitude", 1, 3),
    PlotInfo("Acceleration Magnitude", "m/s/s", "acceleration_magnitude", 2, 1),
    PlotInfo("Ascent Velocity", "m/s", "velocity", 2, 2)
]

def create_graph(plot: PlotInfo) -> dbc.Col:
    return dbc.Col(dcc.Graph(id=plot.data_name, figure=dict(
        data = [dict(x = [], y=[])],
        layout = dict(
            xaxis = dict(range = [-1, 1]),
            yaxis = dict(range = [-1, 1], title=dict(text = plot.label)),
            title = dict(text = plot.fancy_name)
        ),
    )), lg=4, md=12)

app = Dash("KSC TSAT-2B Live Telemetry Monitor", external_stylesheets=[dbc.themes.BOOTSTRAP])
app.layout = [
    dcc.Interval(id="interval", interval=1000), # The interval to update the graphs when we recieve packets
    dcc.Store(id="processed-packets", data=0), # How many packets we have processed so far

    dbc.Container([
        dbc.Row([
            create_graph(plot_info[0]),
            create_graph(plot_info[1]),
            create_graph(plot_info[2]),
        ]),
        dbc.Row([
            create_graph(plot_info[3]),
            create_graph(plot_info[4]),
            dbc.Col(html.Img(src=app.get_asset_url("ksc.png")), lg=4, md=12, style=dict(textAlign="center")),
        ])
    ], fluid=True),
]

@callback(
   Output("processed-packets", "data"),
   Input("interval", "n_intervals"),
   State("processed-packets", "data"),
)
def update_plots(_, processed_packets):
    # Take in new telemetry data
    for plot in plot_info:
        # This is the additional graph data we append
        new_data_x, new_data_y = [], []
        # Look through all the packets we haven't processed yet
        for idx in range(processed_packets, len(telemetry_data)):
            packet = telemetry_data[idx]
            new_data_x.append(packet.frame_count)
            new_data_y.append(getattr(packet, plot.data_name))

        # Setting the extendData property appends the data to the graph
        # We use set_props rather than callback Output to avoid having
        # to reduce code bloat
        set_props(plot.data_name, dict(extendData=[dict(x=[new_data_x], y=[new_data_y])]))

    return len(telemetry_data)

# --------------------[ Main Program ]--------------------

parser = argparse.ArgumentParser(
    description = "TSAT-2B Telemetry Monitor"
)
action_group = parser.add_mutually_exclusive_group(required=True)
action_group.add_argument("--live", metavar="port", help="Recieves live telemetry from a serial ground reciever. Port is usually COMX on Windows and /dev/ttyUSBX or /dev/ttySX on Linux/Mac where X is a number.")
action_group.add_argument("--from-file", metavar="file", help="Reads telemetry data from a file and displays it.")
action_group.add_argument("--test-graphs", action="store_true", help="Debugging utility. Creates random data for graphs to test the display.")
args = parser.parse_args()

running = True
def run():
    if args.test_graphs:
        i = 0

        while running:
            telemetry_data.append(TelemetryPacket(
                i,
                i * 1000,
                0,
                i * 10,
                40*np.sin(i),
                5 + random.random() * 10,
                0
            ))
            i += 1

            time.sleep(1)

    elif args.live:
        ser = serial.Serial(args.live, baudrate=115200)

        while running:
            raw = ser.readline()
            try:
                raw = raw.decode().strip()
                print("Raw:", raw)
                line = bytes([int(i) for i in raw.split(' ')])
                packet = deserialize_packet(line)
                print("Packet:", packet)
                if packet.meta.packet_type == PacketType.TELEMETRY:
                    telemetry_data.append(packet.data)
                print()
            except:
                continue

    elif args.from_file:
        lines = open(args.from_file, 'r').readlines()
        for line in lines:
            data = [float(n) for n in line.strip().split(',')]
            packet = TelemetryPacket(*data)
            telemetry_data.append(packet)

        
t = threading.Thread(target=run)
t.start()

app.run()
# On CTRL-C, app.run() terminates, so lets clean up
running = False
t.join()