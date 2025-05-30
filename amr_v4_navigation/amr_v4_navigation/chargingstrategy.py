import sys
import os
import rclpy
import time
import threading
from threading import Thread
import platform
import select
import datetime
import can
import pdb
from rclpy.node import Node
from amr_v4_msgs_srvs.msg import Charging,Battery
from rclpy.executors import MultiThreadedExecutor

robot_name = (os.getenv('ROBOT_MODEL','amr_x'))

class CANController(Node):
    def __init__(self):
        super().__init__('BatteryNode',namespace=robot_name)
        self.batt_publisher = self.create_publisher(Battery,'/'+robot_name+'/battery_state',10)
        self.charger_subs = self.create_subscription(Charging,'/'+robot_name+'/charger_cmd',self.charger_request,10)
        self.can_interface = 'can1'
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
        self.desired_voltage = 27.45
        self.desired_current = 35.0
        self.sender_thread = None
        self.monitor_thread = None
        self.function_choice = None
        self.start_charge = False
        self.charger_subs
        self.count = 0
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
        self.initialize_can()
        self.threads = []
        self.current_mode = "idle"

    def initialize_can(self):
        try:
            from can.interface import Bus
            self.bus = Bus(channel=self.can_interface, interface='socketcan')
            return True
        except Exception as e:
            print(f"CAN initialization error: {str(e)}")
            return 
        
    def charger_request(self,msg):
        self.count = self.count_publishers('/'+robot_name+'/charger_cmd')
        if msg.start_charging_main:
            self.start_charge = True
        else:
            self.start_charge = False

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
        print(f"the message is :{self.demand_powerstage1}")
        val |= (self.demand_clear_faults & 0x01) << 21
        val |= (cur << 32)
        data = val.to_bytes(7, 'little')
        return data

    def send_0x190_message(self):
        data_190 = self.pack_0x190_message()
        self.send_can_message(0x190, data_190)

    def parse_32X_message(self, data):
        val = int.from_bytes(data, 'little')
        self.last_32X_data = val
        if self.is_wireless_connected():
            #self.print_32X_errors(val)
            pass

    def print_32X_errors(self, val):
        errors = []
        status_state = val & 0x3F

        for bit, name in self.bit_definitions_32x.items():
            if ((val >> bit) & 0x01) == 1:
                errors.append(name)

        if errors or status_state != 0:
            print("0x32X Status:")
            print(f" Status_state: {status_state}")
            if errors:
                print(" Errors/Faults detected:")
                for e in errors:
                    print(f"  - {e}")
            else:
                print(" No errors/faults.")
        else:
            print("0x32X: No errors or faults and Status_state=0")

    def parse_77X_message(self, data):
        val = int.from_bytes(data, 'little')
        rssi = (val >> 48) & 0xFF
        comm_success_rate_raw = (val >> 16) & 0xFFFFFFFF
        comm_success_rate = comm_success_rate_raw * 0.000001
        self.last_77X_data = {
            'success_rate': comm_success_rate,
            'rssi': rssi,
        }
        #print(f"Ctrl_CommSuccessRate   : {comm_success_rate:.6f}")
        #print(f"Ctrl_CommRssi          : {rssi}")

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
            self.print_32X_errors(self.last_32X_data)
            print("Fault detected!")
            if self.is_ovp_fault():
                print("OVP Fault detected, setting PowerStage1=0 before clearing.")
                self.demand_powerstage1 = 0
                self.send_0x190_message()
                time.sleep(0.5)
            print("Clearing faults...")
            self.clear_faults_sequence()

    def print_selected_data_simple(self, received_data):
        message = Battery()
        if len(received_data) < 42:
            print("Received data is shorter than expected (42 bytes). Skipping...")
            return
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
        #fahrenheit_temps = ", ".join(f'{(temp * 9 / 5 + 32):.1f}' for temp in ntc_content)
        current_ma = current * 10
        if current_ma >= 1000:
            current_a = current_ma / 1000
            if current_a.is_integer():
                #print(f"Current         : {int(current_a)}A")
                message.current = float(current_a)
            else:
                #print(f"Current         : {current_a:.1f}A")
                message.current = float(current_a)
        else:
            #print(f"Current         : {current_ma}mA")
            message.current = float(current_ma)
        message.voltage = float(total_voltage) * 10/1000
        
        message.temp = celsius_temps
        message.soc = float(soc)
        message.soh = float(soh)
        #print(f"soc             : {soc}%")
        protection_status = received_data[27] * 256 + received_data[28]
        control_status = received_data[29]
        self.wireless_connected = bool(control_status & 0x80)
        message.wireless_comms = bool(self.wireless_connected)

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
            message.error = False
        else:
            message.error = True
            print("\nProtection Status: Protection(s) triggered")
        message.weak_switch = 'Closed' if control_status & 0x10 else 'Open'
        if self.last_77X_data is not None and self.last_77X_data['rssi'] is not None:
            message.charger_signal = float(self.last_77X_data['rssi'])
        self.batt_publisher.publish(message)
        # print(f"  Weak Switch       : {'Closed' if control_status & 0x10 else 'Open'}")
        # print(f"  Active Balancing  : {'Working' if control_status & 0x20 else 'Disabled'}")
        # print(f"  Weak Signal       : {'True' if control_status & 0x40 else 'False'}")
        # print(f"  Wireless_Comms    : {'Connected' if control_status & 0x80 else 'Disconnected'}")

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
        received_data = bytearray()
        last_print_time = time.time()
        while self.running and not self.stop_event.is_set():
            #print("f is wireless connected",self.is_wireless_connected())
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
                elif (msg.arbitration_id & 0x7F0) == 0x770:
                    self.parse_77X_message(msg.data)
            if self.running is False:
                break
            if time.time() - last_print_time > 5:
               if received_data:
                   pass
                   #print(f"Clearing incomplete data: {received_data}")
               received_data = bytearray()
               last_print_time = time.time()
    
    def function1(self):
        print("Starting monitoring mode (function1)...")
        self.current_mode = "monitoring"

        t1 = threading.Thread(target=self.periodic_task_1s_request, name="1sTask")
        t1.start()
        self.threads.append(t1)

        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            for t in self.threads:
                if t.is_alive():
                    t.join()
            self.threads.clear()
            print("Exiting function1 monitoring mode.")

    def function4(self):
        print("Starting charging mode (function4)...")
        self.current_mode = "charging"

        self.demand_voltage = self.desired_voltage
        self.demand_current = self.desired_current
        self.demand_powerstage1 = 1

        t1 = threading.Thread(target=self.periodic_task_1s_request, name="1sTask")
        t2 = threading.Thread(target=self.periodic_task_200ms_190control, name="200msTask")
        t3 = threading.Thread(target=self.start_monitoring_loop, name="MonitorLoop")

        for t in [t1, t2, t3]:
            t.start()
            self.threads.append(t)

        try:
            print("Active charging requested. Checking wireless connection...")
            while not self.stop_event.is_set():
                self.handle_faults()
                if self.is_wireless_connected() and not self.is_fault():
                    print("Wireless connected and no faults. Charging started.")
                    break
                time.sleep(1)
            t3.join()
        finally:
            self.demand_powerstage1 = 0
            self.demand_voltage = 0
            self.demand_current = 0
            self.stop_event.set()

            for t in self.threads:
                if t.is_alive():
                    t.join()
            self.threads.clear()
            print("Exiting function4 charging mode.")

    def function5(self):
        self.handle_faults()
        self.demand_powerstage1 = 0
        self.demand_voltage = 0
        self.demand_current = 0
        task_1s = threading.Thread(target=self.periodic_task_1s_request)
        task_200ms = threading.Thread(target=self.periodic_task_200ms_190control)
        task_1s.start()
        task_200ms.start()
        try:
            self.start_monitoring_loop()
        finally:
            self.stop_event.set()
            task_1s.join()
            task_200ms.join()
    
    def stop_all_threads(self):
        print("Stopping all threads...")
        self.stop_event.set()
        self.running = False

        for t in self.threads:
            if t.is_alive():
                t.join(timeout=2)  # prevent potential hang

        self.threads.clear()
        self.stop_event.clear()
        self.running = True
        print("All threads stopped.")
        


def main():
    rclpy.init(args=None)
    controller = CANController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor_try = Thread(target=executor.spin,daemon=True)
    executor_try.start()
    last_state = False 
    threading.Thread(target=controller.function1, daemon=True).start() 
    try:
        while rclpy.ok():
            
            if controller.start_charge != last_state and controller.count > 0:
                if controller.start_charge:
                    print(f"Charging is enabled for: {robot_name}.")
                    controller.stop_all_threads()
                    time.sleep(2)
                    threading.Thread(target=controller.function4, daemon=True).start() 
                    
                else:
                    
                    print(f"Charging is disabled: {robot_name}.")
                    controller.stop_all_threads()
                    time.sleep(2)
                    threading.Thread(target=controller.function1, daemon=True).start()
                    last_state = False 

                last_state = controller.start_charge

            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
