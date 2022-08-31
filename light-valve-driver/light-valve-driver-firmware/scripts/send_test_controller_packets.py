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


def main(port, baudrate, num_driver_packets, driver_addr, node_addr, node_data):
    try:
        ser = serial.Serial(port, baudrate)
    except Exception as e:
        print(f"Failed to open serial port with exception {e}")
        return

    print(f"Sending {num_driver_packets} driver packets to driver address {driver_addr}.")

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

        print(f"Each driver packet will hold 16 node_data bytes with address nibble {node_addr_str} and data nibble {node_data_str}")

    # Build the actual 8-bit values that will go to each node, i.e. 4 high bits addr 4 low bits data
    node_data_bytes = []
    for i in range(0, 16):
        node_data_bytes.append(((node_addr_list[i] & 0x0F) << 4) | (node_data_list[i] & 0x0F))

    # Convert to ctypes array to match type in class fields
    node_data_bytes_carray = (uint8_t * len(node_data_bytes))(*node_data_bytes)

    for i in range(0, num_driver_packets):
        print(f"Sending driver packet {i + 1}")

        packet = DriverPacket(0x6, driver_addr, node_data_bytes_carray)
        packet_bytes = bytearray(packet)
        ser.write(packet_bytes)

    # Closing immediately will cut off the bytes still waiting to transmit
    sleep(0.2)
    ser.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port to transmit data out")
    parser.add_argument("-b", "--baudrate", help="Baudrate to use when connecting to serial port", default=115200)
    parser.add_argument("-n", "--num_driver_packets", help="Number of packets to emit from controller to driver board(s)", default=1, choices=range(1, 16), type=int)
    parser.add_argument("-a", "--driver_addr", help="Driver address to send packet(s) to", default=0x1, choices=range(1, 16), type=int)
    parser.add_argument("-x", "--node_addr", help=
            "Node address nibble to use for all node_data within the packet. If not provided, node_data addresses will incremement from 0 to 15", choices=range(1, 16), type=int)
    parser.add_argument("-d", "--node_data", help=
            "Node data nibble to use for all node_data within the packet. If not provided, node_data data will incremement from 0 to 15", choices=range(1, 16), type=int)
    args = parser.parse_args()

    main(args.port, args.baudrate, args.num_driver_packets, args.driver_addr, args.node_addr, args.node_data)
