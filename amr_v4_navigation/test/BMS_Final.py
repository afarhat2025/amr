import socket
import struct
import threading
import time
import sys

class CANFrame:
    def __init__(self, can_id, is_extended=False, is_remote=False, data=None):
        self.can_id = can_id
        self.is_extended = is_extended
        self.is_remote = is_remote
        self.data = data or []

    def pack(self):
        can_id = self.can_id
        if self.is_extended:
            can_id |= socket.CAN_EFF_FLAG
        if self.is_remote:
            can_id |= socket.CAN_RTR_FLAG

        data = bytes(self.data)
        dlc = len(data)

        frame_format = "<IB3x8s"
        return struct.pack(frame_format, can_id, dlc, data)

    @classmethod
    def unpack(cls, packet):
        frame_format = "<IB3x8s"
        can_id, dlc, data = struct.unpack(frame_format, packet)

        is_extended = bool(can_id & socket.CAN_EFF_FLAG)
        is_remote = bool(can_id & socket.CAN_RTR_FLAG)

        can_id &= socket.CAN_EFF_MASK if is_extended else socket.CAN_SFF_MASK
        return cls(can_id, is_extended, is_remote, list(data[:dlc]))


def send_thread(sock):
    while True:
        frame1 = CANFrame(0x1a1, data=[0xDD, 0x0D, 0x03, 0x03, 0x01, 0x00, 0x15, 0xf8]).pack()
        sock.send(frame1)

        frame2 = CANFrame(0x1a1, data=[0x77]).pack()
        sock.send(frame2)

        time.sleep(1)


def receive_thread(sock):
    received_data = bytearray()
    while True:
        packet = sock.recv(16)
        frame = CANFrame.unpack(packet)
        if frame.can_id == 0x2b2:
            received_data.extend(frame.data)
            if len(received_data) == 42:
                print_selected_data_simple(received_data)
                received_data = bytearray()
        else:
            print(f"Received unexpected packet: CAN ID: {frame.can_id:#x}, Data: {[f'{d:#x}' for d in frame.data]}")


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
        interface = 'can1'  # 使用的socketCAN接口名，可以是can0, can1等

        # 创建socketCAN的原始socket
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((interface,))

        sender = threading.Thread(target=send_thread, args=(sock,))
        receiver = threading.Thread(target=receive_thread, args=(sock,))

        sender.start()
        receiver.start()

        sender.join()
        receiver.join()
        #print_selected_data_simple(received_data)
    except (KeyboardInterrupt,SystemError,SystemExit):
        sys.exit()


if __name__ == "__main__":
    main()
