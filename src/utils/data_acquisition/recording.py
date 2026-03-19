#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scheduler_interfaces.srv import GetCurrentPoi
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile
import csv
import os
import json


class PoiTimer(Node):

    def __init__(self):
        super().__init__('poi_timer')

        self.cli = self.create_client(GetCurrentPoi, '/SchedulerComponent/GetCurrentPoi')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /SchedulerComponent/GetCurrentPoi service...')

        self.current_poi = None
        self.start_time = None  # Store simulation time as rclpy.time.Time
        self.tour_started = False
        self.request_pending = False

        self.poi_times = []
        self.total_time = rclpy.time.Duration()  # Use rclpy.time.Duration for total time

        self.csv_file = "tour_times_with_detection.csv"
        self.log_file = "logger.txt"

        self.recoveries = 0
        self.just_abort = 0
        self.aborts = 0
        self.last_recovery_count = 0

        self.timer = self.create_timer(1.0, self.check_poi)

        qos_profile = QoSProfile(depth=10)
        self.feedback_subscriber = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            qos_profile
        )

        self.tf_time = None  # Store the latest time from /tf topic
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            qos_profile
        )

    # -------------------------
    # TIME FORMAT
    # -------------------------

    def format_time(self, seconds):
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes:02d}:{secs:02d}"

    # -------------------------
    # SERVICE CALL
    # -------------------------

    def check_poi(self):

        if self.request_pending:
            return

        self.request_pending = True
        req = GetCurrentPoi.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_poi_response)

    # -------------------------
    # HANDLE RESPONSE
    # -------------------------

    def handle_poi_response(self, future):

        self.request_pending = False

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        poi = (response.poi_name, response.poi_number)

        # -------------------------
        # TOUR FINISHED
        # -------------------------

        if response.poi_name == "madama_start":

            if self.tour_started and self.current_poi is not None and self.tf_time is not None:
                elapsed = self.tf_time - self.start_time
                self.store_poi_time(self.current_poi, elapsed)

            if self.tour_started:

                self.get_logger().info("Tour finished")
                self.get_logger().info(
                    f"Total tour time: {self.format_time(self.total_time.nanoseconds / 1e9)}"
                )

                self.save_csv()

            # reset state
            self.current_poi = None
            self.start_time = None
            self.tour_started = False
            self.poi_times = []
            self.total_time = rclpy.time.Duration()
            self.recoveries = 0
            self.aborts = 0

            return

        self.tour_started = True

        # -------------------------
        # POI CHANGE
        # -------------------------

        if self.current_poi != poi:
            #print("Tour started ")

            if self.current_poi is not None and self.tf_time is not None:
                elapsed = self.tf_time - self.start_time
                self.store_poi_time(self.current_poi, elapsed)

            self.get_logger().info(f'POI changed: {self.current_poi} -> {poi}')

            self.current_poi = poi
            self.start_time = self.tf_time
            self.last_recovery_count = 0
            self.recoveries = 0
            self.aborts = 0

    # -------------------------
    # FEEDBACK CALLBACK
    # -------------------------

    def feedback_callback(self, msg):
        #print("in callback")

        recovery_count = msg.feedback.number_of_recoveries
        #print("Recovery count:", recovery_count)

        if recovery_count == 0 and self.last_recovery_count > 0:
            self.aborts += 1
            self.just_abort = 1
            print("Aborted")

        if recovery_count > self.last_recovery_count:
            if self.just_abort > 0:
                # Add the new recovery count to the previous recoveries after an abort
                self.recoveries += recovery_count
                #print("recoveries: ", self.recoveries)
                self.just_abort = 0  # Reset aborts after accounting for recoveries
            else:
                self.recoveries += recovery_count - self.last_recovery_count
                #print("recoveries: ", self.recoveries)

        self.last_recovery_count = recovery_count

        # Log the current state for debugging
        #print(f"Total recoveries: {self.recoveries}, Aborts: {self.aborts}")

        # with open(self.log_file, 'a') as log:
        #     log.write(json.dumps({
        #         "feedback": {
        #             "current_poi": self.current_poi,
        #             "number_of_recoveries": msg.feedback.number_of_recoveries,
        #         }
        #     }, indent=4) + "\n")

    # -------------------------
    # STORE POI TIME
    # -------------------------

    def store_poi_time(self, poi, elapsed):

        poi_name, poi_number = poi

        self.total_time = rclpy.time.Duration(nanoseconds=self.total_time.nanoseconds + elapsed.nanoseconds)
        formatted = self.format_time(elapsed.nanoseconds / 1e9)

        self.poi_times.append({
            "poi_name": poi_name,
            "poi_number": poi_number,
            "time": formatted,
            "recoveries": self.recoveries,
            "aborts": self.aborts
        })

        self.get_logger().info(
            f"POI {poi_name} ({poi_number}) time: {formatted}, Recoveries: {self.recoveries}, Aborts: {self.aborts}"
        )

    # -------------------------
    # SAVE CSV
    # -------------------------

    def save_csv(self):

        file_exists = os.path.isfile(self.csv_file)

        with open(self.csv_file, 'a', newline='') as csvfile:

            writer = csv.writer(csvfile)

            if not file_exists:
                writer.writerow(["poi_name", "poi_number", "time_mmss", "recoveries", "aborts"])

            for poi in self.poi_times:
                writer.writerow([
                    poi["poi_name"],
                    poi["poi_number"],
                    poi["time"],
                    poi["recoveries"],
                    poi["aborts"]
                ])

            writer.writerow([
                "TOTAL",
                "",
                self.format_time(self.total_time.nanoseconds / 1e9),
                "",
                ""
            ])

            writer.writerow([
                "############## END ##############"
            ])

        self.get_logger().info(f"Results appended to {self.csv_file}")

    # -------------------------
    # TF CALLBACK
    # -------------------------

    def tf_callback(self, msg):
        # Extract the time from the /tf topic
        #self.get_logger().info(f"TF message received: {msg}")
        self.tf_time = rclpy.time.Time(seconds=msg.transforms[0].header.stamp.sec, nanoseconds=msg.transforms[0].header.stamp.nanosec)


def main(args=None):

    rclpy.init(args=args)

    node = PoiTimer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
