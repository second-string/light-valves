import sys
import argparse
import serial
import ctypes
uint8_t = ctypes.c_uint8

from time import sleep

class DriverPacket(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [("start_bits", uint8_t, 4),
                ("addr_bits", uint8_t, 4),
                ("node_data", uint8_t * 16)]

class NodePacket:
    def __init__(self, addr, data):
        if addr > 0xF:
            raise Exception("NodePacket addr can only be between 0x0 and 0xF")
        if data > 0xF:
            raise Exception("NodePacket data can only be between 0x0 and 0xF")

        self.addr = addr
        self.data = data


pattern_choices = ["all_on", "all_off", "blink",  "blink_drivers", "incrementing", "one_by_one", "one_by_one_rev", "full_fade", "moving_fade"]


def build_driver_packet(driver_addr, node_packet_list, debug_output=False):
    # Build the actual 8-bit values that will go to each node, i.e. 4 high bits addr 4 low bits data
    node_data_bytes = []
    packets_len = len(node_packet_list)
    for i in range(0, 16):
        if i < packets_len:
            node_data_bytes.append((node_packet_list[i].addr << 4) | node_packet_list[i].data)
        else:
            node_data_bytes.append(i << 4)

    # Convert to ctypes array to match type in class fields
    node_data_bytes_carray = (uint8_t * len(node_data_bytes))(*node_data_bytes)

    if debug_output:
        print(f"Sending packet to driver address {driver_addr}.")
        print([f"{packet.addr}, {packet.data}" for packet in node_packet_list])
    return DriverPacket(0x6, driver_addr, node_data_bytes_carray)

def send_driver_packet(ser, driver_addr, node_packet_list):
    driver_packet = build_driver_packet(driver_addr, node_packet_list, True)
    packet_bytes = bytearray(driver_packet)
    print(packet_bytes)
    ser.write(packet_bytes)
    sleep(0.002)


def manual(ser, num_drivers, node_addr, node_data):
    node_addr_str = ""
    if (node_addr):
        node_addr_str = str(node_addr)
        node_addr_list = [node_addr] * 16
    else:
        node_addr_str = "incrementing range(0, 16)"
        node_addr_list = range(0, 16)

    node_data_str = ""
    node_data_list = []
    if (node_data):
        node_data_str = str(node_data)
        node_data_list = [node_data] * 16
    else:
        node_data_str = "incrementing range(0, 16)"
        node_data_list = range(0, 16)

    node_packets = []
    for i in range(0, 16):
        node_packets.append(NodePacket(node_addr_list[i], node_data_list[i]))

    print(f"Driver packet has 16 node_data bytes with address nibble {node_addr_str} and data nibble {node_data_str}")
    for d in range(0, num_drivers):
        send_driver_packet(ser, d, node_packets)


def pattern(ser, num_drivers, pattern):
    print(f"Displaying pattern '{pattern}'. If pattern repeats indefinitely, CTL+C to stop")

    pattern_change_interval_s = 0.1
    node_packets = [NodePacket(i, 0x0) for i in range(0, 0x10)]

    if pattern == "all_on":
        for node_packet in node_packets:
            node_packet.data = 0xF
        for d in range(0, num_drivers):
            send_driver_packet(ser, d, node_packets)
    elif pattern == "all_off":
        for node_packet in node_packets:
            node_packet.data = 0x0
        for d in range(0, num_drivers):
            send_driver_packet(ser, d, node_packets)
    elif pattern == "blink":
        on = True
        while 1:
            for node_packet in node_packets:
                node_packet.data = 0xF if on else 0x0
            for d in range(0, num_drivers):
                send_driver_packet(ser, d, node_packets)

            on = not on
            sleep(pattern_change_interval_s * 5)
    elif pattern == "blink_drivers":
        on = True
        while 1:
            for d in range(0, num_drivers):
                for node_packet in node_packets:
                    node_packet.data = 0xF
                send_driver_packet(ser, d, node_packets)
                sleep(pattern_change_interval_s * 5)

                for node_packet in node_packets:
                    node_packet.data = 0x0
                send_driver_packet(ser, d, node_packets)
    elif pattern == "incrementing":
        for i, node_packet in enumerate(node_packets):
            node_packet.data = i
        for d in range(0, num_drivers):
            send_driver_packet(ser, d, node_packets)
    elif pattern == "one_by_one":
        node_on_index = 0
        while 1:
            for d in range(0, num_drivers):
                for n in range(0, 0x10):
                    node_packets[n].data = 0xF
                    send_driver_packet(ser, d, node_packets)
                    sleep(pattern_change_interval_s)
                    node_packets[n].data = 0x0

                # Turn off last node in that driver universe before starting on next driver
                send_driver_packet(ser, d, node_packets)
    elif pattern == "one_by_one_rev":
        node_on_index = 0
        while 1:
            for d in range(num_drivers - 1, -1, -1):
                for n in range(0, 0x10):
                    node_packets[n].data = 0xF
                    send_driver_packet(ser, d, node_packets)
                    sleep(pattern_change_interval_s)
                    node_packets[n].data = 0x0

                # Turn off last node in that driver universe before starting on next driver
                send_driver_packet(ser, d, node_packets)
    elif pattern == "full_fade":
        node_data = 0
        fade_dir = 1 # fading darker or lighter
        while 1:
            if node_packets[0].data == 0xF:
                fade_dir = -1
            elif node_packets[0].data == 0x0:
                fade_dir = 1
            for node_packet in node_packets:
                node_packet.data += fade_dir

            for d in range(0, num_drivers):
                send_driver_packet(ser, d, node_packets)
            sleep(pattern_change_interval_s)
    elif pattern == "moving_fade":
        current_node_index = 0
        moving_dir = 1 # moving up or down node addresses
        while 1:
            node_packets[current_node_index].data = 0xF
            for i in range(1, 0x10):
                if current_node_index + i <= 0xF:
                    node_packets[current_node_index + i].data = 0xF - i
                if current_node_index - i >= 0x0:
                    node_packets[current_node_index - i].data = 0xF - i

            if current_node_index == 0xF:
                moving_dir = -1
            elif current_node_index == 0x0:
                moving_dir = 1
            current_node_index += moving_dir

            for d in range(0, num_drivers):
                send_driver_packet(ser, d, node_packets)
            sleep(pattern_change_interval_s)
    else:
        raise Exception("Unsupported pattern!")



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port to transmit data out")
    parser.add_argument("-b", "--baudrate", help="Baudrate to use when connecting to serial port", default=115200)
    parser.add_argument("-d", "--num_drivers", help="Number of driver addrs to send packet(s) to", default=0x1, choices=range(1, 5), type=int)
    subs = parser.add_subparsers(help="Choose what mode to execute the script in", dest="mode")

    # Subparser for all 'manual' mode args
    manual_parser = subs.add_parser("manual", help="Provide direct node addr/data for debugging")
    manual_parser.add_argument("-x", "--node_addr", help=
            "Node address nibble to use for all node_data within the packet. If not provided, all 16 nodes will be addresssed", choices=range(1, 16), type=int)
    manual_parser.add_argument("-y", "--node_data", help=
            "Node data nibble to use for all node_data within the packet. If not provided, node_data will match each node address its sent to", choices=range(1, 16), type=int)

    # Subparser for all 'pattern' mode args
    pattern_parser = subs.add_parser("pattern", help="Provide a pattern to display across all connected nodes")
    pattern_parser.add_argument("pattern_choice", help="Pattern to display", choices=pattern_choices)

    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baudrate)
    except Exception as e:
        print(f"Failed to open serial port with exception: {e}")
        sys.exit(1)

    if args.mode == "manual":
        manual(ser, args.num_drivers, args.node_addr, args.node_data)
    elif args.mode == "pattern":
        pattern(ser, args.num_drivers, args.pattern_choice)
    else:
        raise Exception("Must provide 'mode' arg of either 'manual' or 'pattern'")

    # Closing immediately will cut off the bytes still waiting to transmit
    sleep(0.2)
    ser.close()
