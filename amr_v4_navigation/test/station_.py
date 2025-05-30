# !/usr/bin/env python
import configparser
import random
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import RPi.GPIO as GPIO
from datetime import datetime
import socket
from typing import List
from threading import Thread
from pymongo import MongoClient
# from dataclasses_json import dataclass_json
from dataclasses import dataclass
# import json
from evdev import InputDevice
from select import select
from datetime import datetime, timedelta
import time


@dataclass
class StationStatus:
    """Holds all status of Station"""
    station_id: int = 0  # ID of Station
    last_rfid: int = 0  # Last scanned
    pickup_rfid: int = 0  # Will not be used
    dropoff_rfid: int = 0  # Will not be used
    last_pickup_time: datetime = datetime.now()  # When the most recent pickup has occurred
    in_place: bool = False  # Rack in place
    enabled: bool = True  # Station is turned on or not
    rfid: int = 0  # The rfid that the station is at
    path: List[int] = None  # Also not sure what this is used for
    cycle_time: datetime = datetime.now()  # Not really sure what this is used for
    distance: int = 0  # Distance from "Home" to station?
    median_time: int = 500  # Time between racks
    assigned: int = 0  # Which AGV is assigned to this station
    in_progress: bool = False  # Will be unused
    pull = None  # Unused?
    name: str = ""  # Name of station
    allow_prod: bool = True  # If production is allowed to print another label


def get_db():
    try:
        config = configparser.ConfigParser()
        config.read('/home/ubuntu/fisher_agc/AGCROS/.ini')
        db_uri = config.get('AGVP3', 'MONGO_URI')
        db_ns = config.get('AGVP3', 'MONGO_NS')
        db = MongoClient(
            db_uri
        )[db_ns]
        return db
    except Exception as err:
        print("Error: {0}".format(err))


def update_station_db(db, station):
    update_doc = {
        "_id": station.station_id,
        "last_rfid": station.last_rfid,
        "dropoff_rfid": station.dropoff_rfid,
        "pickup_rfid": station.pickup_rfid,
        "last_pickup_time": station.last_pickup_time,
        "in_place": station.in_place,
        "enabled": station.enabled,
        "rfid": station.rfid,
        "path": station.path,
        "cycle_time": station.cycle_time,
        "distance": station.distance,
        "median_time": station.median_time,
        "assigned": station.assigned,
        "in_progress": station.in_progress,
        "pull": station.pull,
        "name": station.name,
        "allow_prod": station.allow_prod
    }
    try:
        return db.stations.update_one(
            {"_id": station.station_id},
            {"$set": update_doc},
            upsert=True
        )
    except Exception as err:
        print("Error: {0}".format(err))
        return False


def get_status(db, station):
    try:
        doc = db.stations.find_one(
            {"name": station.name}
        )

        station.station_id = doc["_id"]
        station.last_rfid = doc["last_rfid"]
        station.dropoff_rfid = doc["dropoff_rfid"]
        station.pickup_rfid = doc["pickup_rfid"]
        station.last_pickup_time = doc["last_pickup_time"]
        station.in_place = doc["in_place"]
        station.enabled = doc["enabled"]
        station.rfid = doc["rfid"]
        station.path = doc["path"]
        station.cycle_time = doc["cycle_time"]
        station.distance = doc["distance"]
        station.median_time = doc["median_time"]
        station.assigned = doc["assigned"]
        station.in_progress = doc["in_progress"]
        station.pull = doc["pull"]
        station.name = doc["name"]
        station.allow_prod = doc["allow_prod"]

    except Exception as err:
        print("Error: {0}".format(err))
        return None


def update_one_field(db, station: StationStatus, field: str):
    update_doc = {
        field: getattr(station, field)
    }
    try:
        return db.stations.update_one(
            {"name": station.name},
            {"$set": update_doc},
            upsert=True
        )
    except Exception as err:
        print("Error: {0}".format(err))
        return False


GPIO.setmode(GPIO.BOARD) 
hostname = socket.gethostname()
name = hostname.split("-", 1)[0]
inPlace = False 
out_pins = (11, 13, 15, 16)  
in_pins = (18,22)  
red_light = 11
green_light = 13
blue_light = 15
yellow_light = 16
sensor_front = 18
sensor_back = 22
my_station = StationStatus(name=name)
my_db = get_db()
get_status(my_db, my_station)
print(my_station)
part_number = ""
part_status = ""
keys = "X^1234567890XXXXqwertzuiopXXXXasdfghjklXXXXXyxcvbnmXXXXXXXXXXXXXXXXXXXXXXX"
r_keyboard = True

good_shift = 0
bad_shift = 0
first_shift = 12   # Time First shift starts in UTC
second_shift = 20  # Time Second shift starts in UTC
third_shift = 4    # Time Third shift starts in UTC
scanned = False
productions = []
cart_detected = False
set_in_progress = False


class ProductionPosted(Node):

    def __init__(self):
        super().__init__('get_production_info')
        self.subscription = self.create_subscription(
            String,
            'mach2',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        global name
        global productions
        msg_list = msg.data.split(';', -1)  # Split message from Mach2 on colins
        if len(msg_list) < 4:
            if msg_list[1] == "Get":
                productions.clear()
                cursor = my_db.production.find({'workcenter': name}).sort('datetime', -1).limit(1)
                for doc in cursor:
                    p_number = doc["part_no"]
                    s_number = doc["serial_no"]
                    p_status = doc["status"]
                    productions.append((p_number, s_number, p_status))
            else:
                productions.clear()
        else:
            if name == msg_list[0]:  # Only listen for messages about this station
                p_number = msg_list[1]  # Part number is first after workcenter
                s_number = msg_list[2]  # Next is serial number
                p_status = msg_list[3]  # Finally part status (OK or Rework)
                productions.append((p_number, s_number, p_status))


class TakeRack(Node):

    def __init__(self):
        super().__init__('take_rack')
        self.publisher_ = self.create_publisher(String, 'rack_ready', 10)
        timer_period = random.uniform(0.8, 1.2)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lastrfid = 0
        self.reset_start_time = None
        self.reset_delay = timedelta(seconds=120)

    def timer_callback(self):
        msg = String()
        global name
        global part_status
        global rfid
        global part_number
        global serial_number
        global scanned
        global cart_detected

        get_status(my_db, my_station)
        sensor_front_high = GPIO.input(sensor_front) == GPIO.HIGH
        sensor_back_high = GPIO.input(sensor_back) == GPIO.HIGH

        if my_station.allow_prod:
            if sensor_front_high and sensor_back_high:

                my_station.in_place = True
                cart_detected = True
                update_one_field(my_db, my_station, "in_place")
                my_station.in_progress = True
                update_one_field(my_db, my_station, "in_progress")
                GPIO.output(green_light, 0)
                GPIO.output(red_light, 0)
                GPIO.output(blue_light, 1)
                GPIO.output(yellow_light, 0)
                msg.data = '%s,Good' % name
                self.reset_start_time = None

            elif (sensor_front_high != sensor_back_high) and cart_detected:
                if self.reset_start_time is None:
                    self.reset_start_time = datetime.now()

                if datetime.now() - self.reset_start_time >= self.reset_delay:
                    my_station.in_place = False
                    update_one_field(my_db, my_station, "in_place")
                    my_station.in_progress = False
                    update_one_field(my_db, my_station, "in_progress")
                    GPIO.output(green_light, 1)
                    GPIO.output(red_light, 0)
                    GPIO.output(blue_light, 0)
                    GPIO.output(yellow_light, 0)
                    cart_detected = False
                    msg.data = '%s,False' % name
                else:
                    my_station.in_place = False
                    update_one_field(my_db, my_station, "in_place")
                    my_station.in_progress = False
                    update_one_field(my_db, my_station, "in_progress")
                    GPIO.output(green_light, 0)
                    GPIO.output(red_light, 1)
                    GPIO.output(blue_light, 0)
                    GPIO.output(yellow_light, 0)
                    msg.data = '%s,False' % name
            else:
                GPIO.output(green_light, 1)
                GPIO.output(red_light, 0)
                GPIO.output(blue_light, 0)
                GPIO.output(yellow_light, 0)
                my_station.in_place = False
                my_station.in_progress = False
                cart_detected = False
                update_one_field(my_db, my_station, "in_place")
                update_one_field(my_db, my_station, "last_rfid")
                msg.data = '%s,False' % name
        elif my_station.allow_prod is False and my_station.in_place is True:
            my_station.in_place = True
            cart_detected = True
            update_one_field(my_db, my_station, "in_place")
            my_station.in_progress = True
            update_one_field(my_db, my_station, "in_progress")
            GPIO.output(green_light, 0)
            GPIO.output(red_light, 0)
            GPIO.output(blue_light, 1)
            GPIO.output(yellow_light, 0)
            msg.data = '%s,Good' % name
            self.reset_start_time = None
        else:
            GPIO.output(red_light, 0)
            GPIO.output(green_light, 0)
            GPIO.output(blue_light, 0)
            GPIO.output(yellow_light, 1)
            msg.data = '%s,False' % name
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        time.sleep(0.5)


def print_serial():  # Logging function REMOVE CLEARING SERIAL NUMBER ON IMPLEMENTATION
    global serial_number
    global part_number
    global scanned
    print(scanned)
    doc = my_db.production.find_one(
        {"serial_no": serial_number}
    )
    part_number = doc["part_no"]
    print("Serial number scanned: %s\nWhich has Part number: %s" % (serial_number, part_number))
    scanned = True


class Keyboard:

    def __init__(self):
        print("IM OPEING KEYBAORD")
        self.dev = InputDevice(
            '/dev/input/by-id/usb-Symbol_Technologies__Inc__2008_Symbol_Bar_Code_Scanner::EA_21043010559656-event-kbd')

    def scanner_checker(self):
        global serial_number
        global r_keyboard
        scanned = ""
        while r_keyboard:
            r, w, x = select([self.dev], [], [])
            for event in self.dev.read():
                if event.type == 1 and event.value == 1:
                    if event.code == 42:
                        pass
                    elif event.code == 28:
                        serial_number = scanned
                        scanned = ""
                        if serial_number[0] == "s":
                            serial_number = serial_number[1:]
                        serial_number = serial_number.upper()
                        print_serial()


                    else:
                        print("%s with code %s" % (keys[event.code], str(event.code)))
                        scanned += keys[event.code]


class ProgressChecker(Thread):
    def run(self) -> None:
        prev_progress = False
        while True:
            doc = my_db.stations.find_one(
                {"name": name}
            )
            if prev_progress and not doc["in_progress"]:
                # productions.pop(0)  # Remove first element in list
                my_station.last_pickup_time = datetime.utcnow()
                update_one_field(my_db, my_station, "last_pickup_time")
            if doc["in_progress"]:
                prev_progress = True
            else:
                prev_progress = False
            time.sleep(1)


def main():
    print("Got to main")
    rclpy.init(args=None)
    rack_publisher = TakeRack()  # ROS2 publisher to let AGV know that rack is in place or not
    mach2_listener = ProductionPosted()
    executor = MultiThreadedExecutor()
    start_time = None
    end_time = None
    try:
        for p in out_pins: #follow R,G,B,Y
            GPIO.setup(p, GPIO.OUT)

        for p in in_pins:
            GPIO.setup(p, GPIO.IN,
                       pull_up_down=GPIO.PUD_DOWN)
        executor.add_node(rack_publisher)
        executor.add_node(mach2_listener)
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        my_progress = ProgressChecker(daemon=True)
        my_progress.start()
        print("ROS nodes started")
        GPIO.output(green_light, 1)  # Make pendant light green

        global good_shift
        global bad_shift

        while rclpy.ok():
            time.sleep(2)
            print("looping for nothing.")

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        GPIO.cleanup()
        executor.shutdown()
        rack_publisher.destroy_node()
        mach2_listener.destroy_node()
        rclpy.shutdown()
        rfid.run = False
        global r_keyboard
        r_keyboard = False


if __name__ == '__main__':
    main()