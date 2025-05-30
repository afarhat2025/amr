import socket
import struct
import threading
import time
import os
import can

class USBCAN:
    def __init__(self, can_id, is_extended=False, is_remote=False, data=None):
        self.can_id = can_id
        self.is_extended = is_extended
        self.is_remote = is_remote
        self.data = data or []

    def pack(self):
        frame_format = 0x80000000 if self.is_extended else 0
        can_id = self.can_id | frame_format
        dlc = len(self.data)
        packet = struct.pack("=IB3x8s", can_id, dlc, bytes(self.data))
        return packet

    @classmethod
    def unpack(cls, packet):
        can_id, dlc, data = struct.unpack("=IB3x8s", packet)
        is_extended = bool(can_id & 0x80000000)
        can_id &= 0x1FFFFFFF  # Mask to get the actual CAN ID
        data = list(data[:dlc])
        return cls(can_id, is_extended, False, data)

def send_packet(sock, packet):
    sock.send(packet)

def receive_packet(sock):
    packet = sock.recv(16)
    return USBCAN.unpack(packet)

def send_thread(sock):
    while True:
        packet1 = USBCAN(0x1a1, data=[0xDD, 0x0D, 0x03, 0x03, 0x01, 0x00, 0x15, 0xf8]).pack()
        send_packet(sock, packet1)

        packet2 = USBCAN(0x1a1, data=[0x77]).pack()
        send_packet(sock, packet2)

        time.sleep(1)

def print_selected_data_simple(received_data):
    total_voltage = received_data[12] * 256 + received_data[13]
    current = received_data[14] * 256 + received_data[15]
    soc = received_data[18] * 256 + received_data[19]
    ntc_count = received_data[30]
    ntc_content = [(received_data[31 + i * 2] * 256 + received_data[32 + i * 2]) / 10 - 273.1 for i in range(ntc_count)]
    soh = received_data[36]
    protection_status = received_data[27] * 256 + received_data[28]
    temp_protection_info = received_data[37] * 256 + received_data[38]

    print("Selected Data:")
    print(f"Total Voltage: {total_voltage * 10} mV")
    print(f"Current: {current * 10} mA")
    print(f"NTC Temperature: {[f'{temp:.1f}' for temp in ntc_content]} °C")
    print(f"SOC: {soc / 100}%")
    print(f"SOH: {soh}%")

    protection_triggered = False
    if protection_status & 0x0001:
        print("  Single Cell Overvoltage Protection: Active")
        protection_triggered = True
    if protection_status & 0x0002:
        print("  Single Cell Undervoltage Protection: Active")
        protection_triggered = True
    if protection_status & 0x0010:
        print("  Single Cell Level 1 Overvoltage Protection: Active")
        protection_triggered = True
    if protection_status & 0x0020:
        print("  Single Cell Level 1 Undervoltage Protection: Active")
        protection_triggered = True
    if protection_status & 0x0040:
        print("  Weak Switch: On")
        protection_triggered = True
    if protection_status & 0x0080:
        print("  Level 2 Overcurrent Protection: Active")
        protection_triggered = True
    if protection_status & 0x0100:
        print("  Short Circuit Protection: Active")
        protection_triggered = True
    if protection_status & 0x2000:
        print("  Charge Overcurrent Level 1 Protection: Active")
        protection_triggered = True
    if protection_status & 0x4000:
        print("  Discharge Overcurrent Level 1 Protection: Active")
        protection_triggered = True
    if protection_status & 0x8000:
        print("  Cell Voltage Difference Alarm: Active")
        protection_triggered = True

    if temp_protection_info & 0x0001:
        print("  Cell Discharge Overtemperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0002:
        print("  Cell Charge Overtemperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0004:
        print("  Cell Discharge Low Temperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0008:
        print("  Cell Charge Low Temperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0010:
        print("  MOS Overtemperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0020:
        print("  MOS Low Temperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0040:
        print("  Environment Overtemperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0080:
        print("  Environment Discharge Low Temperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0100:
        print("  Environment Charge Low Temperature Protection: Active")
        protection_triggered = True
    if temp_protection_info & 0x0200:
        print("  Temperature Sensor Failure: Active")
        protection_triggered = True

    if not protection_triggered:
        print("Protection Status: No protection triggered")


def main():
    try:
        bus = can.interface.Bus(channel='can1', bustype='socketcan')
        
    except OSError:
        print("Cannot find can1 interface.")
        return

    sender = threading.Thread(target=send_thread, args=(bus,))
    sender.start()

    received_data = bytearray()
    while True:
        message = receive_packet(bus)
        if message:
            frame = USBCAN.unpack(message)
            if frame.can_id == 0x2b2:
                received_data.extend(frame.data)
                if len(received_data) == 42:
                    print_selected_data_simple(received_data)
                    received_data = bytearray()
            else:
                print(f"Received unexpected packet: CAN ID: {frame.can_id:#x}, Data: {[f'{d:#x}' for d in frame.data]}")


if __name__ == "__main__":
    main()