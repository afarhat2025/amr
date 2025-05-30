import sys
import subprocess
import importlib
import struct
import serial.tools.list_ports
import time
import threading


def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        importlib.import_module(import_name)
    except ImportError:
        print(f"{package} not found. Installing...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])
    finally:
        globals()[import_name] = importlib.import_module(import_name)


install_and_import('pyserial', 'serial')


class USBCAN:
    def __init__(self, can_id, is_extended=False, is_remote=False, data=None):
        self.can_id = can_id
        self.is_extended = is_extended
        self.is_remote = is_remote
        self.data = data or []

    def pack(self):
        header = 0xaa
        frame_type = 0x20 if self.is_extended else 0x00
        frame_format = 0x10 if self.is_remote else 0x00
        frame_dlc = len(self.data) & 0x0f
        frame_type_dlc = 0xc0 | frame_type | frame_format | frame_dlc

        if self.is_extended:
            can_id_bytes = struct.pack('<I', self.can_id)
        else:
            can_id_bytes = struct.pack('<H', self.can_id)

        data_bytes = bytes(self.data)
        end_code = 0x55

        packet = bytes([header, frame_type_dlc]) + can_id_bytes + data_bytes + bytes([end_code])
        return packet

    @classmethod
    def unpack(cls, packet):
        header = packet[0]
        frame_type_dlc = packet[1]
        is_extended = bool(frame_type_dlc & 0x20)
        is_remote = bool(frame_type_dlc & 0x10)
        frame_dlc = frame_type_dlc & 0x0f

        if is_extended:
            can_id = struct.unpack('<I', packet[2:6])[0]
            data_start = 6
        else:
            can_id = struct.unpack('<H', packet[2:4])[0]
            data_start = 4

        data = list(packet[data_start:data_start + frame_dlc])
        end_code = packet[-1]
        if header == b'\xaa':
            if end_code == 0x55:
                return cls(can_id, is_extended, is_remote, data)


def find_com_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def send_packet(ser, packet):
    ser.write(packet)


def receive_packet(ser):
    while True:
        header = ser.read(1)
        if header == b'\xaa':
            frame_type_dlc = ser.read(1)[0]
            is_extended = bool(frame_type_dlc & 0x20)
            frame_dlc = frame_type_dlc & 0x0f

            if is_extended:
                can_id = struct.unpack('<I', ser.read(4))[0]
            else:
                can_id = struct.unpack('<H', ser.read(2))[0]

            data = list(ser.read(frame_dlc))
            end_code = ser.read(1)[0]

            if end_code == 0x55:
                return USBCAN(can_id, is_extended, False, data)


def send_thread(ser):
    while True:
        packet1 = USBCAN(0x1a1, data=[0xDD, 0x0D, 0x03, 0x03, 0x01, 0x00, 0x15, 0xf8]).pack()
        send_packet(ser, packet1)

        packet2 = USBCAN(0x1a1, data=[0x77]).pack()
        send_packet(ser, packet2)

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


def initialize_device(ser, can_baud_rate, frame_type, filter_id, mask_id, can_mode, auto_retransmit):
    init_message = [
        0xAA, 0x55,
        0x12,
        can_baud_rate,  # CAN Baud Rate
        # 0x01(1Mbps) 0x02（800kbps）0x03（500kbps），0x04（400kbps），0x05（250kbps），
        # 0x06（200kbps），0x07（125kbps），0x08（100kbps）， 0x09（50kbps）
        # 0x0a（20kbps） 0x0b（10kbps） 0x0c（5kbps）
        frame_type,  # Frame type
        # 0x01 standard frame, 0x02 extended frame
        (filter_id >> 24) & 0xFF, (filter_id >> 16) & 0xFF, (filter_id >> 8) & 0xFF, filter_id & 0xFF,  # Filter ID
        (mask_id >> 24) & 0xFF, (mask_id >> 16) & 0xFF, (mask_id >> 8) & 0xFF, mask_id & 0xFF,  # Block ID
        can_mode,  # CAN mode
        # 0x00 - - Normal Mode, 0x01 - - Silent Mode,
        # 0x02 - - Loopback Mode, 0x03 - - Loopback Silent Mode
        auto_retransmit,  # Auto resending
        # 0x00- Automatic resend 0x01- Prohibit automatic resend
        0x00, 0x00, 0x00, 0x00  # reserve
    ]
    # 计算校验码
    checksum = sum(init_message[2:19]) & 0xFF
    init_message.append(checksum)

    ser.write(bytes(init_message))
    time.sleep(1)


def main():
    ports = find_com_ports()
    if not ports:
        print("No available COM ports found.")
        return

    print("Available COM ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    port_index = int(input("Select COM port index: "))
    if port_index < 0 or port_index >= len(ports):
        print("Invalid index.")
        return

    port = ports[port_index]
    baudrate = 115200       # Serial Port Baud Rate

    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)

    can_baud_rate = 0x05  # 250kbps
    frame_type = 0x01  # Standard Frame
    filter_id = 0x00000000
    mask_id = 0x00000000
    can_mode = 0x00  # Normal mode
    auto_retransmit = 0x00  # Auto resending

    initialize_device(ser, can_baud_rate, frame_type, filter_id, mask_id, can_mode, auto_retransmit)

    sender = threading.Thread(target=send_thread, args=(ser,))
    sender.start()

    received_data = bytearray()
    while True:
        frame = receive_packet(ser)
        if frame:
            if frame.can_id == 0x2b2:
                received_data.extend(frame.data)
                if len(received_data) == 42:
                    print_selected_data_simple(received_data)
                    received_data = bytearray()
            else:
                print(f"Received unexpected packet: CAN ID: {frame.can_id:#x}, Data: {[f'{d:#x}' for d in frame.data]}")

    ser.close()


if __name__ == "__main__":
    main()
