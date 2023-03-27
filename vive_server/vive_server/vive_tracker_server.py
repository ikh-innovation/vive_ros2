"""
OpenVr based Vive tracker server
"""

import argparse
import json
import logging
import logging.handlers
import socket
from multiprocessing import Queue, Process, Pipe
from pathlib import Path
from typing import List
from typing import Optional
import yaml
import numpy as np
import scipy.spatial.transform as transform
import time
import os

from base_server import Server
from models import ViveDynamicObjectMessage, ViveDynamicControllerMessage, ViveStaticObjectMessage, Configuration
from triad_openvr import TriadOpenVR


def construct_controller_socket_msg(data: ViveDynamicControllerMessage) -> str:
    """
    Send vive controller message to socket

    Args:
        data: ViveTracker Message to send

    Returns:
        message in string to send

    """
    json_data = json.dumps(data.json(), sort_keys=False)
    json_data = "&" + json_data
    json_data = json_data + "\r"  # * (512 - len(json_data))
    return json_data

def construct_tracker_socket_msg(data: ViveDynamicObjectMessage) -> str:
    """
    Send vive tracker message to socket

    Args:
        data: ViveTracker Message to send

    Returns:
        message in string to send

    """
    json_data = json.dumps(data.json(), sort_keys=False)
    json_data = "&" + json_data
    json_data = json_data + "\r"  # * (512 - len(json_data))
    return json_data


class ViveDeviceServer(Server):
    """
    Defines a UDP vive tracker server that constantly "shout out" messages at (HOST, PORT)

    Utilizes OpenVR as its interaction with SteamVR. For hardware setup, please see this tutorial:
    http://help.triadsemi.com/en/articles/836917-steamvr-tracking-without-an-hmd

    """

    def __init__(self, port: int, pipe: Pipe, logging_queue: Queue, buffer_length: int = 1024):
        """
        Initialize socket and OpenVR
        
        Args:
            port: desired port to open
            logging_queue: handler with where to send logs
            buffer_length: maximum buffer (tracker_name) that it can listen to at once
        """
        super(ViveDeviceServer, self).__init__(port)
        self.logger = logging.getLogger("ViveDeviceServer")
        self.logger.addHandler(logging.handlers.QueueHandler(logging_queue))
        self.logger.setLevel(logging.INFO)
        self.pipe = pipe
        self.config = Configuration()

        self.socket = self.initialize_socket()
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.reconnect_triad_vr()

        self.buffer_length = buffer_length

    def run(self, type):
        """
        Initialize a server that runs forever.

        This server can be put into a multi-process module to run concurrently with other servers.

        This server will listen for client's request for a specific tracker's name

        It will compute that tracker's information

        It will then send that information
        Returns:
            None
        """
        self.logger.info(f"Starting server at {self.ip}:{self.port}")
        self.logger.info("Connected VR devices: \n###########\n" + str(self.triad_openvr) + "###########")
        # Main server loop
        while True:
            self.triad_openvr.poll_vr_events()
            messages = {"state": {}}
            if type == "tracker":
                try:
                    tracker_name, addr = self.socket.recvfrom(self.buffer_length)
                    tracker_name = tracker_name.decode()
                    tracker_key = self.resolve_name_to_key(tracker_name)
                    if tracker_key in self.get_tracker_keys():
                        message = self.poll_tracker(tracker_key=tracker_key)
                        messages["state"][tracker_key] = message
                        if message is not None:
                            socket_message = construct_tracker_socket_msg(data=message)
                            self.socket.sendto(socket_message.encode(), addr)
                    else:
                        self.logger.error(f"Tracker {tracker_name} with key {tracker_key} not found")
                except socket.timeout:
                    self.logger.info("Did not receive connection for tracker from client")
                except Exception as e:
                    self.logger.error(e)
            else:
                try:
                    controller_name, addr = self.socket.recvfrom(self.buffer_length)
                    controller_name = controller_name.decode()
                    controller_key = self.resolve_name_to_key(controller_name)
                    if controller_key in self.get_controller_keys():
                        message = self.poll_controller(controller_key=controller_key)
                        messages["state"][controller_key] = message
                        if message is not None:
                            socket_message = construct_controller_socket_msg(data=message)
                            self.socket.sendto(socket_message.encode(), addr)
                    else:
                        self.logger.error(f"Controller {controller_name} with key {controller_key} not found")
                except socket.timeout:
                    self.logger.info("Did not receive connection for controller from client")
                except Exception as e:
                    self.logger.error(e)

    def resolve_name_to_key(self, name):
        """
        Takes in a name that is either assigned to a device serial number
        or a key. Note that the name should not resemble the keys automatically assigned
        to devices.
        """
        keys = list(self.config.name_mappings.keys())
        values = list(self.config.name_mappings.values())
        for i in range(len(values)):
            if values[i] == name:
                serial = keys[i]
                for device_key in self.get_device_keys():
                    if self.get_device(device_key).get_serial() == serial:
                        return device_key
                return keys[i]
        return name

    def poll_tracker(self, tracker_key) -> Optional[ViveDynamicObjectMessage]:
        """
        Polls tracker message by name

        Note:
            Server will attempt to reconnect if tracker name is not found.

        Args:
            tracker_key: the vive tracker message intended to poll

        Returns:
            ViveTrackerMessage if tracker is found, None otherwise.
        """
        tracker = self.get_device(key=tracker_key)
        if tracker is not None:
            message: Optional[ViveDynamicObjectMessage] = self.create_dynamic_message(device=tracker,
                                                                                      device_key=tracker_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def poll_controller(self, controller_key) -> Optional[ViveDynamicControllerMessage]:
        """
        Polls controller message by name

        Note:
            Server will attempt to reconnect if controller name is not found.

        Args:
            controller_key: the vive controller message intended to poll

        Returns:
            ViveControllerMessage if controller is found, None otherwise.
        """
        controller = self.get_device(key=controller_key)
        if controller is not None:
            message: Optional[ViveDynamicControllerMessage] = self.create_dynamic_controller_message(device=controller,
                                                                                      device_key=controller_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def poll_tracking_reference(self, tracking_reference_key) -> Optional[ViveStaticObjectMessage]:
        """
        Polls tracking reference message by name

        Note:
            Server will attempt to reconnect if tracker name is not found.

        Args:
            tracking_reference_key: the vive tracking reference intended to poll

        Returns:
            ViveTrackerMessage if tracker is found, None otherwise.
        """
        tracking_reference = self.get_device(key=tracking_reference_key)
        if tracking_reference is not None:
            message: Optional[ViveStaticObjectMessage] = self.create_static_message(device=tracking_reference,
                                                                                    device_key=tracking_reference_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def get_device(self, key):
        """
        Given tracker name, find the tracker instance

        Args:
            key: desired tracker's name to find

        Returns:
            tracker instance if found, None otherwise
        """
        return self.triad_openvr.devices.get(key, None)

    def create_dynamic_message(self, device, device_key) -> Optional[ViveDynamicObjectMessage]:
        """
        Create dynamic object message given device and device name

        Note:
            it will attempt to reconnect to OpenVR if conversion or polling from device went wrong.

        Args:
            device: tracker instance
            device_key: the device's name corresponding to this tracker

        Returns:
            Vive dynamic message if this is a successful conversion, None otherwise

        """
        try:
            x, y, z, qw, qx, qy, qz = device.get_pose_quaternion()
            vel_x, vel_y, vel_z = device.get_velocity()
            p, q, r = device.get_angular_velocity()

            serial = device.get_serial()
            device_name = device_key if serial not in self.config.name_mappings else self.config.name_mappings[serial]
            message = ViveDynamicObjectMessage(valid=True, x=x, y=y, z=z,
                                               qx=qx, qy=qy, qz=qz, qw=qw,
                                               vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
                                               p=p, q=q, r=r,
                                               device_name=device_name,
                                               serial_num=serial)
            return message
        except OSError as e:
            self.logger.error(f"OSError: {e}. Need to restart Vive Tracker Server")
            self.reconnect_triad_vr()
        except Exception as e:
            self.logger.error(f"Exception {e} has occurred, this may be because device {device} "
                              f"is either offline or malfunctioned")
            self.reconnect_triad_vr()
            return None


    def create_dynamic_controller_message(self, device, device_key) -> Optional[ViveDynamicControllerMessage]:
        """
        Create dynamic object message given device and device name

        Note:
            it will attempt to reconnect to OpenVR if conversion or polling from device went wrong.

        Args:
            device: tracker instance
            device_key: the device's name corresponding to this tracker

        Returns:
            Vive dynamic message if this is a successful conversion, None otherwise

        """
        try:
            x, y, z, qw, qx, qy, qz = device.get_pose_quaternion()
            vel_x, vel_y, vel_z = device.get_velocity()
            p, q, r = device.get_angular_velocity()

            serial = device.get_serial()
            device_name = device_key if serial not in self.config.name_mappings else self.config.name_mappings[serial]
            cont_inputs = device.get_controller_inputs()

            message = ViveDynamicControllerMessage(valid=True, x=x, y=y, z=z,
                                               qx=qx, qy=qy, qz=qz, qw=qw,
                                               vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
                                               p=p, q=q, r=r, 
                                               menu_butt=cont_inputs['menu_button'],
                                               trig=cont_inputs['trigger'],
                                               pad_x=cont_inputs['trackpad_x'],
                                               pad_y=cont_inputs['trackpad_y'],
                                               pad_touch=cont_inputs['trackpad_touched'],
                                               pad_butt=cont_inputs['trackpad_pressed'],
                                               ul_butt=cont_inputs['ulButtonPressed'],
                                               ul_touch=cont_inputs['ulButtonTouched'],
                                               grip_butt=cont_inputs['grip_button'],
                                               device_name=device_name,
                                               serial_num=serial)
            return message
        except OSError as e:
            self.logger.error(f"OSError: {e}. Need to restart Vive Tracker Server")
            self.reconnect_triad_vr()
        except Exception as e:
            self.logger.error(f"Exception {e} has occurred, this may be because device {device} "
                              f"is either offline or malfunctioned")
            self.reconnect_triad_vr()
            return None

    def create_static_message(self, device, device_key) -> Optional[ViveStaticObjectMessage]:
        """
        Create tracker message given device and device name

        Note:
            it will attempt to reconnect to OpenVR if conversion or polling from tracker went wrong.

        Args:
            device: device instance
            device_key: the device's name corresponding to this tracker

        Returns:
            Vive static message if this is a successful conversion, None otherwise

        """
        try:
            x, y, z, qw, qx, qy, qz = device.get_pose_quaternion()
            serial = device.get_serial()
            device_name = device_key if serial not in self.config.name_mappings else self.config.name_mappings[serial]
            message = ViveStaticObjectMessage(valid=True, x=x, y=y, z=z,
                                              qx=qx, qy=qy, qz=qz, qw=qw,
                                              device_name=device_name,
                                              serial_num=serial)
            return message
        except OSError as e:
            self.logger.error(f"OSError: {e}. Need to restart Vive Tracker Server")
            self.reconnect_triad_vr()
        except Exception as e:
            self.logger.error(f"Exception {e} has occurred, this may be because device {device} "
                              f"is either offline or malfunctioned")
            self.reconnect_triad_vr()
            return None

    def reconnect_triad_vr(self, debug=False):
        """
        Attempt to reconnect to TriadOpenVR

        Notes:
            this method will automatically assign self.triad_openvr

        Args:
            debug: **deprecated flag

        Returns:
            openvr instance
        """
        del self.triad_openvr
        self.triad_openvr = TriadOpenVR()

        if debug:
            self.logger.debug(
                f"Trying to reconnect to OpenVR to refresh devices. "
                f"Devices online:")
            self.logger.info(self.triad_openvr.devices)

    def get_tracker_keys(self) -> List[str]:
        """
        Get a list of trackers

        Returns:
            list of tracker names

        """
        return self.get_device_keys(filters=["tracker"])

    def get_tracking_reference_keys(self) -> List[str]:
        """
        Get a list of tracking references (base stations)

        Returns:
            list of references names

        """
        return self.get_device_keys(filters=["reference"])

    def get_controller_keys(self) -> List[str]:
        """
        Get a list of controllers

        Returns:
            list of controller names

        """
        return self.get_device_keys(filters=["controller"])

    def get_device_keys(self, filters=None) -> List[str]:
        result = []
        for device_name in self.triad_openvr.devices.keys():
            if filters is None:
                result.append(device_name)
            else:
                for s in filters:
                    if s in device_name:
                        result.append(device_name)
        return result

def run_tracker_server(port: int, pipe: Pipe, logging_queue: Queue):
    vive_tracker_server = ViveDeviceServer(port=port, pipe=pipe, logging_queue=logging_queue)
    vive_tracker_server.run("tracker")

def run_controller_server(port: int, pipe: Pipe, logging_queue: Queue):
    vive_controller_server = ViveDeviceServer(port=port, pipe=pipe, logging_queue=logging_queue)
    vive_controller_server.run("controller")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Vive Device server')
    parser.add_argument('--controller_port', default=8000, help='port to broadcast controller data on')
    parser.add_argument('--tracker_port', default=8001, help='port to broadcast tracker data on')
    args = parser.parse_args()

    logger_queue = Queue()
    server_conn = Pipe()
    string_formatter = logging.Formatter(fmt='%(asctime)s|%(name)s|%(levelname)s|%(message)s', datefmt="%H:%M:%S")
    
    p1 = Process(target=run_tracker_server, args=(args.tracker_port, server_conn, logger_queue))
    p1.start()
    p2 = Process(target=run_controller_server, args=(args.controller_port, server_conn, logger_queue))
    p2.start()
    try:
        # This should be updated to be a bit cleaner
        while True:
            print(string_formatter.format(logger_queue.get()))
    finally:
        p1.kill()
        p2.kill()
    

