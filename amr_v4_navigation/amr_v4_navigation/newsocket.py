import sys
import time
import threading
import platform
import select
import datetime
import termios
import tty
import can
import pdb

class CANController:
    def __init__(self, can_interface):
        self.can_interface = can_interface
        self.running = False
        self.stop_event = threading.Event()
        self.bus = None
        self.demand_voltage = 0.0
        self.demand_current = 0.0
        self.demand_clear_faults = 0
        self.demand_powerstage1 = 0
        self.last_32X_data = 0
        self.last_5FX_data = 0
        self.last_77X_data = None
        self.desired_voltage = 27.50
        self.desired_current = 50.0
        self.sender_thread = None
        self.monitor_thread = None
        self.function_choice = None
        self.bit_definitions_32x = {
            6: "Status_DeratingT",
            7: "Fault_Disconnect",
            8: "Status_BattMinVDetect",
            9: "Status_Fan",
            10: "Status_Power",
            11: "Status_Interlock",
            12: "Fault",
            13: "Alert_Fan",
            14: "Alert_Temperature",
            15: "Fault_BDD",
            16: "Fault_Fan",
            17: "Fault_Interlock",
            18: "Fault_OCP_SW",
            19: "Fault_OTP_HW",
            20: "Fault_OTP_SW",
            21: "Fault_OVP_HW",
            22: "Fault_OVP_SW",
            23: "Alert_AC",
            24: "Fault_WD_Comms",
            25: "Fault_WD_Reset",
            26: "Fault_PFC",
            27: "Fault_PFC_Comms",
            28: "Fault_PowerStage",
            29: "Fault_OCP_HW",
            30: "Fault_MemoryCorruption",
            31: "Fault_RevProt"
        }

    def initialize_can(self):
        try:
            from can.interface import Bus
            self.bus = Bus(channel=self.can_interface, interface='socketcan')
            return True
        except Exception as e:
            print(f"CAN initialization error: {str(e)}")
            return False

    def send_can_message(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"CAN send error: {str(e)}")

    def send_9byte_command(self, can_id, data_9byte):
        frame1 = data_9byte[0:8]
        frame2 = data_9byte[8:9]
        self.send_can_message(can_id, frame1)
        self.send_can_message(can_id, frame2)

    def pack_0x190_message(self):
        vol = int(self.demand_voltage * 1000) & ((1 << 20) - 1)
        cur = int(self.demand_current * 1000) & ((1 << 18) - 1)
        val = 0
        val |= vol
        val |= (self.demand_powerstage1 & 0x01) << 20
        val |= (self.demand_clear_faults & 0x01) << 21
        val |= (cur << 32)
        data = val.to_bytes(7, 'little')
        return data

    def send_0x190_message(self):
        data_190 = self.pack_0x190_message()
        self.send_can_message(0x190, data_190)

    def get_key(self):
        if platform.system() == 'Windows':
            return None
        else:
            if select.select([sys.stdin], [], [], 0)[0]:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch.lower()
            return None

    def parse_32X_message(self, data):
        val = int.from_bytes(data, 'little')
        self.last_32X_data = val
        self.print_32X_errors(val)

    def print_32X_errors(self, val):
        errors = []
        status_state = val & 0x3F  # bits 0-5

        for bit, name in self.bit_definitions_32x.items():
            if ((val >> bit) & 0x01) == 1:
                errors.append(name)

        if errors or status_state != 0:
            #print("0x32X Status:")
            #print(f" Status_state: {status_state}")
            if errors:
                #print(" Errors/Faults detected:")
                for e in errors:
                    print(f"  - {e}")
            else:
                print(" No errors/faults.")
        else:
            print("0x32X: No errors or faults and Status_state=0")




    def parse_5FX_message(self, data):
        val = int.from_bytes(data, 'little')
        self.last_5FX_data = val

    def parse_77X_message(self, data):
        val = int.from_bytes(data, 'little')
        rssi = (val >> 48) & 0xFF
        comm_success_rate_raw = (val >> 16) & 0xFFFFFFFF
        comm_success_rate = comm_success_rate_raw * 0.000001
        lqi = (val >> 56) & 0xFF
        comm_id = (val >> 8) & 0xFF
        comm_channel = val & 0xFF
        self.last_77X_data = {
            'channel': comm_channel,
            'id': comm_id,
            'success_rate': comm_success_rate,
            'rssi': rssi,
            'lqi': lqi
        }
        # print("\n---------- RSSI Status (0x77X) ----------")
        # print(f"Ctrl_CommChannel       : {comm_channel}")
        # print(f"Ctrl_CommId            : {comm_id}")
        # print(f"Ctrl_CommSuccessRate   : {comm_success_rate:.6f}")
        print(f"Ctrl_CommRssi          : {rssi}")
        #print(f"Ctrl_CommLQI           : {lqi}")
        # print("-----------------------------------------\n")

    def is_fault(self):
        return ((self.last_32X_data >> 12) & 0x01) == 1

    def is_ovp_fault(self):
        ovp_hw = ((self.last_32X_data >> 21) & 0x01) == 1
        ovp_sw = ((self.last_32X_data >> 22) & 0x01) == 1
        return ovp_hw or ovp_sw

    def is_wireless_connected(self):
        return ((self.last_5FX_data >> 7) & 0x01) == 1

    def clear_faults_sequence(self):
        self.demand_clear_faults = 0
        self.send_0x190_message()
        time.sleep(0.5)
        self.demand_clear_faults = 1
        self.send_0x190_message()
        time.sleep(0.5)
        self.demand_clear_faults = 0

    def handle_faults(self):
        if self.is_fault():
            #self.print_32X_errors(self.last_32X_data)
            #print("Fault detected!")
            if self.is_ovp_fault():
                print("OVP Fault detected, setting PowerStage1=0 before clearing.")
                self.demand_powerstage1 = 0
                self.send_0x190_message()
                time.sleep(0.5)
            print("Clearing faults...")
            self.clear_faults_sequence()

    def print_selected_data_simple(self, received_data):
        if len(received_data) < 42:
            print("Received data is shorter than expected (42 bytes). Skipping...")
            return
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        total_voltage = received_data[12] * 256 + received_data[13]
        current = received_data[14] * 256 + received_data[15]
        soc = received_data[18] * 256 + received_data[19]
        soh = received_data[36]
        ntc_count = received_data[30]
        max_ntc_possible = (len(received_data) - 31) // 2
        ntc_count = min(ntc_count, max_ntc_possible)
        ntc_content = []
        for i in range(ntc_count):
            index1 = 31 + i * 2
            index2 = 32 + i * 2
            if index2 >= len(received_data):
                break
            ntc_value = (received_data[index1] * 256 + received_data[index2]) / 10 - 273.1
            ntc_content.append(ntc_value)
        celsius_temps = ", ".join(f'{temp:.1f}' for temp in ntc_content)
        fahrenheit_temps = ", ".join(f'{(temp * 9 / 5 + 32):.1f}' for temp in ntc_content)
        current_ma = current * 10
        print(f"\nSelected Data: {current_time}")
        print(f"Total Voltage   : {total_voltage * 10} mV")
        if current_ma >= 1000:
            current_a = current_ma / 1000
            if current_a.is_integer():
                print(f"Current         : {int(current_a)}A")
            else:
                print(f"Current         : {current_a:.1f}A")
        else:
            print(f"Current         : {current_ma}mA")
        #print(f"NTC Temperature : {celsius_temps} °C / {fahrenheit_temps} °F")
        print(f"SOC             : {soc}%")
        print(f"SOH             : {soh}%")
        protection_status = received_data[27] * 256 + received_data[28]
        control_status = received_data[29]
        self.wireless_connected = bool(control_status & 0x80)

        temp_protection_info = received_data[37] * 256 + received_data[38]

        protection_triggered = False
        # Check protection_status
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
        if protection_status & 0x0080:
            print("  Discharge Overcurrent Level 2 Protection: Active")
            protection_triggered = True
        if protection_status & 0x0100:
            print("  Short Circuit Protection: Active")
            protection_triggered = True
        if protection_status & 0x0200:
            print("  Charger No Communication: Active")
            protection_triggered = True
        if protection_status & 0x0400:
            print("  Charger Weak Signal: Active")
            protection_triggered = True
        if protection_status & 0x0800:
            print("  Charger OVP: Active")
            protection_triggered = True
        if protection_status & 0x1000:
            print("  Charger OCP: Active")
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

        # Check temp_protection_info
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
        if temp_protection_info & 0x0400:
            print("  Charger Primary Overtemperature: Active")
            protection_triggered = True
        if temp_protection_info & 0x0800:
            print("  Charger Secondary Overtemperature: Active")
            protection_triggered = True
        if not protection_triggered:
            print("\nProtection Status: No protection triggered")
        else:
            print("\nProtection Status: Protection(s) triggered")
        # Check control_status
        # print("\nControl Status:")
        # print(f"  Charge MOS        : {'Open' if control_status & 0x01 else 'Closed'}")
        # print(f"  Discharge MOS     : {'Open' if control_status & 0x02 else 'Closed'}")
        # print(f"  Charger           : {'Working' if control_status & 0x04 else 'Standby'}")
        # print(f"  Charger Enable    : {'Enabled' if control_status & 0x08 else 'Disabled'}")
        # print(f"  Weak Switch       : {'Closed' if control_status & 0x10 else 'Open'}")
        # print(f"  Active Balancing  : {'Working' if control_status & 0x20 else 'Disabled'}")
        # print(f"  Weak Signal       : {'True' if control_status & 0x40 else 'False'}")
        # print(f"  Wireless_Comms    : {'Connected' if control_status & 0x80 else 'Disconnected'}")

    def stop_monitoring(self):
        self.running = False
        self.stop_event.set()
        if self.sender_thread:
            self.sender_thread.join()

    def periodic_task_1s_request(self):
        req_cmd = bytes([0xDD,0x0D,0x03,0x03,0x01,0x00,0x15,0xf8,0x77])
        while not self.stop_event.is_set():
            self.send_9byte_command(0x1a1, req_cmd)
            time.sleep(1)

    def periodic_task_200ms_190control(self):
        while not self.stop_event.is_set():
            self.handle_faults()
            self.send_0x190_message()
            time.sleep(0.2)

    def start_monitoring_loop(self):
        self.running = True
        self.stop_event.clear()
        print("\nMonitoring started. Press 'q' to quit...\n")
        received_data = bytearray()
        last_print_time = time.time()
        while self.running and not self.stop_event.is_set():
            msg = self.bus.recv(0.1)
            if msg is not None:
                if msg.arbitration_id == 0x2b2:
                    received_data.extend(msg.data)
                    while len(received_data) >= 42:
                        chunk = received_data[:42]
                        received_data = received_data[42:]
                        self.print_selected_data_simple(chunk)
                        last_print_time = time.time()
                elif (msg.arbitration_id & 0x7F0) == 0x320:
                    self.parse_32X_message(msg.data)
                elif (msg.arbitration_id & 0x7F0) == 0x5F0:
                    self.parse_5FX_message(msg.data)
                elif (msg.arbitration_id & 0x7F0) == 0x770:
                    self.parse_77X_message(msg.data)
            #if self.running is False:
            #    break
            # key = self.get_key()
            # if key == 'q':
            #     print("\nStopping monitoring...")
            #     self.running = False
            #     break
            if time.time() - last_print_time > 5:
                if received_data:
                    print(f"Clearing incomplete data: {received_data}")
                received_data = bytearray()
                last_print_time = time.time()

    def function1(self):
        task_thread = threading.Thread(target=self.periodic_task_1s_request)
        task_thread.start()
        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            task_thread.join()

    def function2(self):
        charge_req = bytes([0xDD,0x0D,0x03,0x12,0xBE,0xCD,0xB5,0x19,0x77])
        self.send_9byte_command(0x1a1, charge_req)
        task_thread = threading.Thread(target=self.periodic_task_1s_request)
        task_thread.start()
        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            task_thread.join()

    def function3(self):
        stop_req = bytes([0xDD,0x0D,0x03,0x13,0xBD,0xCE,0x84,0x08,0x77])
        self.send_9byte_command(0x1a1, stop_req)
        task_thread = threading.Thread(target=self.periodic_task_1s_request)
        task_thread.start()
        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            task_thread.join()

    def function4(self):
        task_1s = threading.Thread(target=self.periodic_task_1s_request,daemon=True)
        task_1s.start()
        monitor_thread = threading.Thread(target=self.start_monitoring_loop,daemon=True)
        monitor_thread.start()
        try:
            print("Function 4: Active charging started. Checking wireless connection...")
            self.demand_voltage = self.desired_voltage
            self.demand_current = self.desired_current
            self.demand_powerstage1 = 0
            self.demand_powerstage1 = 1
            task_200ms = threading.Thread(target=self.periodic_task_200ms_190control)
            task_200ms.start()
            while not self.stop_event.is_set():
                self.handle_faults()
                if self.is_wireless_connected() and not self.is_fault():
                    print("Wireless connected and no faults. Charging ongoing.")
                    break
                time.sleep(1)
            monitor_thread.join()
        finally:
            self.stop_event.set()
            task_1s.join()
            task_200ms.join()

    def function5(self):
        self.demand_powerstage1 = 0
        self.demand_voltage = 0
        self.demand_current = 0
        self.stop_event.set()
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join()
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join()
        task_1s = threading.Thread(target=self.periodic_task_1s_request,daemon=True)
        task_200ms = threading.Thread(target=self.periodic_task_200ms_190control,daemon=True)
        task_1s.start()
        task_200ms.start()
        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            task_1s.join()
            task_200ms.join()

    def stop_monitoring(self):
        """
        Stops the CAN monitoring process and any running threads gracefully.
        """
        # Set the stop flag to notify threads to terminate
        self.running = False
        self.stop_event.set()

        # Safely join threads if they are running
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join()
            self.sender_thread = None
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join()
            self.monitor_thread = None

        print("CAN monitoring has been stopped.")

def main():
    while True:
        print("\nMenu:")
        print("1. Data monitoring (Function1)")
        print("2. Start charging once + monitoring (Function2)")
        print("3. Stop charging once + monitoring (Function3)")
        print("4. Active Charging (Function4)")
        print("5. Active Stop Charging (Function5)")
        print("6. Exit (Function6)")
        choice = input("Select option (4-6): ")
        if choice == '6':
            break
        print("\nAvailable CAN interfaces:")
        try:
            with open('/proc/net/can/dev', 'r') as f:
                interfaces = [line.split()[0] for line in f.readlines() if 'can' in line]
        except:
            interfaces = ['can0']
        if not interfaces:
            interfaces = ['can0']
        for i, interface in enumerate(interfaces):
            print(f"{i}: {interface}")
        try:
            iface_index = int(input("Select CAN interface index: "))
            if not (0 <= iface_index < len(interfaces)):
                print("Invalid interface index")
                continue
        except ValueError:
            print("Invalid input")
            continue
        selected_iface = interfaces[iface_index]
        print(f"Selected interface: {selected_iface}")
        #pdb.set_trace()
        controller = CANController(can_interface='can0')
        if not controller.initialize_can():
            print("Failed to initialize CAN. Exiting.")
            continue
        if choice == '1':
            controller.function1()
        elif choice == '2':
            controller.function2()
        elif choice == '3':
            controller.function3()
        if choice == '4':
            controller.function4()
        elif choice == '5':
            controller.function5()

if __name__ == "__main__":
    main()
