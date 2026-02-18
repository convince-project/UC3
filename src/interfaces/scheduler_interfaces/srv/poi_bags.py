#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scheduler_interfaces.srv import GetCurrentPoi
from datetime import datetime
import os
import time

import subprocess

class PoiBagRecorder(Node):

    def __init__(self):
        super().__init__('poi_bag_recorder')

        self.cli = self.create_client(GetCurrentPoi, '/SchedulerComponent/GetCurrentPoi')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /SchedulerComponent/GetCurrentPoi service...')

        self.timer = self.create_timer(1.0, self.check_poi)

        self.current_poi = None

        self.start = False 

        self.base_bag_dir = os.path.expanduser('~/poi_bags')
        os.makedirs(self.base_bag_dir, exist_ok=True)

    def check_poi(self):
        req = GetCurrentPoi.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_poi_response)

    def handle_poi_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        poi = (response.poi_name, response.poi_number)

        if response.poi_name == "madama_start" :
            if self.start : 
                self.stop_bag()
                self.get_logger().info("Tour finished")
            return

        self.start = True

        if self.current_poi != poi:
            self.get_logger().info(f'POI changed: {self.current_poi} -> {poi}')
            if response.poi_name == "sala_delle_feste" : 
                self.start_bag(response.poi_name, response.poi_number)
            else : self.stop_bag()
            self.current_poi = poi

        '''
        if self.current_poi != poi:
            self.get_logger().info(f'POI changed: {self.current_poi} -> {poi}')
            self.stop_bag()
            self.start_bag(response.poi_name, response.poi_number)
            self.current_poi = poi
        '''

    def start_bag(self, poi_name, poi_number):
        poi_name = poi_name.replace(' ', '_')
        bag_name = f'{poi_name}_{poi_number}_sim2'
        bag_path = os.path.join(self.base_bag_dir, bag_name)

        self.get_logger().info(f'Start recording bag: {bag_path}')

        cmd = [
            'ros2', 'bag', 'record',
            '--all',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '-o', bag_path
        ]

        self.bag_process = subprocess.Popen(cmd)
        self.get_logger().info('Bag recording started (compressed)')

    def stop_bag(self):
        if hasattr(self, 'bag_process') and self.bag_process:
            self.get_logger().info('Stopping bag recording')
            self.bag_process.terminate()
            self.bag_process.wait()
            self.bag_process = None

    def stop_bag(self):
        if hasattr(self, 'bag_process') and self.bag_process:
            self.get_logger().info('Stopping bag recording')
            self.bag_process.terminate()
            self.bag_process.wait()
            self.bag_process = None

def main(args=None):
    rclpy.init(args=args)
    node = PoiBagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_bag()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()