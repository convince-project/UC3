import string
import time
import math
from matplotlib.pylab import float32
from rclpy.impl import rcutils_logger
import yaml
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from simulated_plan_interfaces.action import Plan
from simulated_plan_interfaces.msg import ExecutePoi, PoiCompleted

from blackboard_interfaces.srv import GetIntBlackboard

import sys
import traceback
from threading import Thread, Lock
from rclpy.executors import Executor, SingleThreadedExecutor
from std_srvs.srv import Trigger


class SimulatedPlanComponent(Node):

    def __init__(self, 
                 time_per_navigation_poi=30.0,
                 time_per_explain_poi=40.0, 
                 time_per_question_poi=20.0, 
                 battery_drainage_per_poi=5.0):
        super().__init__('simulated_plan_component')
        self.declare_parameter('time_per_navigation_poi', float(time_per_navigation_poi))
        self.declare_parameter('time_per_explain_poi', float(time_per_explain_poi))
        self.declare_parameter('time_per_question_poi', float(time_per_question_poi))
        self.declare_parameter('battery_drainage_per_poi', float(battery_drainage_per_poi))

        self.declare_parameter('battery_safety_margin', 10.0)
        self.declare_parameter('low_battery_threshold', 35.0)
        self.declare_parameter('critical_battery_threshold', 20.0)
        self.declare_parameter('moderate_time_pressure_ratio', 1.1)
        self.declare_parameter('high_time_pressure_ratio', 1.3)

        self.time_per_navigation_poi = float(self.get_parameter('time_per_navigation_poi').value)
        self.time_per_explain_poi = float(self.get_parameter('time_per_explain_poi').value)
        self.time_per_question_poi = float(self.get_parameter('time_per_question_poi').value)
        self.battery_drainage_per_poi = float(self.get_parameter('battery_drainage_per_poi').value)

        self.battery_safety_margin = float(self.get_parameter('battery_safety_margin').value)
        self.low_battery_threshold = float(self.get_parameter('low_battery_threshold').value)
        self.critical_battery_threshold = float(self.get_parameter('critical_battery_threshold').value)
        self.moderate_time_pressure_ratio = float(self.get_parameter('moderate_time_pressure_ratio').value)
        self.high_time_pressure_ratio = float(self.get_parameter('high_time_pressure_ratio').value)

        self.get_logger().info(
            "Planner params loaded: nav=%.1f explain=%.1f question=%.1f "
            "battery_drain_per_poi=%.1f" % (
                self.time_per_navigation_poi,
                self.time_per_explain_poi,
                self.time_per_question_poi,
                self.battery_drainage_per_poi,
            )
        )


    def execute_plan(self, goal_handle):
        self.get_logger().info("Executing plan...")
        # Simulate plan execution
        time.sleep(5)  # Simulate time taken to execute the plan
        goal_handle.succeed()
        result = Plan.Result()
        result.success = True
        return result
    
    # PoiCompleted[] pois_completed
    # float32 current_tour_time
    # float32 max_tour_time
    # int32 remaining_pois
    # float32 battery_percentage
    # ---
    # ExecutePoi[] next_pois_actions
    # ---

    # ExecutePoi
    # bool skip_questions
    # bool skip_poi

    # PoiCompleted
    # bool is_overtime
    # string cause
    def plan(self, pois_completed, current_tour_time, max_tour_time, remaining_pois, battery_percentage     ):
        self.get_logger().info("Planning next actions...")
        next_pois_actions = []
        total_remaining_pois = max(0, int(remaining_pois))
        if total_remaining_pois == 0:
            return next_pois_actions

        full_poi_time = (
            self.time_per_navigation_poi
            + self.time_per_explain_poi
            + self.time_per_question_poi
        )
        min_poi_time_without_questions = (
            self.time_per_navigation_poi
            + self.time_per_explain_poi
        )

        time_left = max(0.0, float(max_tour_time) - float(current_tour_time))
        estimated_full_time = float(total_remaining_pois) * full_poi_time
        estimated_min_time = float(total_remaining_pois) * min_poi_time_without_questions
        time_pressure_ratio = estimated_full_time / max(time_left, 1e-3)
        required_time_saving = max(0.0, estimated_full_time - time_left)

        skip_questions_count = 0
        if required_time_saving > 0.0 and self.time_per_question_poi > 0.0:
            skip_questions_count = min(
                total_remaining_pois,
                int(math.ceil(required_time_saving / self.time_per_question_poi)),
            )

        time_saved_by_questions = skip_questions_count * self.time_per_question_poi
        remaining_saving_after_questions = max(0.0, required_time_saving - time_saved_by_questions)

        skip_poi_count = 0
        if remaining_saving_after_questions > 0.0 and full_poi_time > 0.0:
            skip_poi_count = min(
                total_remaining_pois,
                int(math.ceil(remaining_saving_after_questions / full_poi_time)),
            )

        battery = max(0.0, min(100.0, float(battery_percentage)))
        battery_needed_for_all = (
            float(total_remaining_pois) * self.battery_drainage_per_poi
            + self.battery_safety_margin
        )
        battery_shortage = max(0.0, battery_needed_for_all - battery)
        if self.battery_drainage_per_poi > 0.0 and battery_shortage > 0.0:
            skip_poi_count = max(
                skip_poi_count,
                int(math.ceil(battery_shortage / self.battery_drainage_per_poi)),
            )

        if battery < self.critical_battery_threshold:
            skip_questions_count = max(skip_questions_count, total_remaining_pois)
            skip_poi_count = max(skip_poi_count, int(math.ceil(total_remaining_pois * 0.4)))
        elif battery < self.low_battery_threshold:
            skip_questions_count = max(skip_questions_count, int(math.ceil(total_remaining_pois * 0.5)))

        if time_pressure_ratio >= self.high_time_pressure_ratio:
            skip_questions_count = max(skip_questions_count, int(math.ceil(total_remaining_pois * 0.75)))
        elif time_pressure_ratio >= self.moderate_time_pressure_ratio:
            skip_questions_count = max(skip_questions_count, int(math.ceil(total_remaining_pois * 0.35)))

        if estimated_min_time > time_left and min_poi_time_without_questions > 0.0:
            skip_poi_count = max(
                skip_poi_count,
                int(math.ceil((estimated_min_time - time_left) / min_poi_time_without_questions)),
            )

        overtime_nav_causes = 0
        overtime_question_causes = 0
        for poi in pois_completed:
            if not poi.is_overtime:
                continue

            cause = poi.cause.lower()
            if any(keyword in cause for keyword in ["nav", "move", "reach", "path", "obstacle"]):
                overtime_nav_causes += 1
            if any(keyword in cause for keyword in ["question", "answer", "dialog", "speak", "talk"]):
                overtime_question_causes += 1

        if overtime_nav_causes > 0:
            skip_poi_count = max(
                skip_poi_count,
                min(total_remaining_pois, int(math.ceil(overtime_nav_causes * 0.5))),
            )
        if overtime_question_causes > 0:
            skip_questions_count = max(
                skip_questions_count,
                min(total_remaining_pois, int(math.ceil(overtime_question_causes * 0.7))),
            )

        skip_poi_count = min(total_remaining_pois, skip_poi_count)
        skip_questions_count = min(total_remaining_pois, skip_questions_count)

        for idx in range(total_remaining_pois):
            action = ExecutePoi()
            action.skip_poi = idx < skip_poi_count
            action.skip_questions = action.skip_poi or (idx < skip_questions_count)
            next_pois_actions.append(action)

        self.get_logger().info(
            "Plan summary: remaining_pois=%d, skip_poi=%d, skip_questions=%d, "
            "time_left=%.2f, time_pressure=%.2f, battery=%.2f" % (
                total_remaining_pois,
                skip_poi_count,
                skip_questions_count,
                time_left,
                time_pressure_ratio,
                battery,
            )
        )

        return next_pois_actions


def usage():
    print(
        "SimulatedPlanComponent usage:\n"
        "  ros2 run simulated_plan_component simulated_plan_component [--ros-args -p <name>:=<value> ...]\n\n"
        "Common planner parameters:\n"
        "  time_per_navigation_poi      (float, default: 30.0)\n"
        "  time_per_explain_poi         (float, default: 40.0)\n"
        "  time_per_question_poi        (float, default: 20.0)\n"
        "  battery_drainage_per_poi     (float, default: 5.0)\n"
        "  battery_safety_margin        (float, default: 10.0)\n"
        "  low_battery_threshold        (float, default: 35.0)\n"
        "  critical_battery_threshold   (float, default: 20.0)\n"
        "  moderate_time_pressure_ratio (float, default: 1.1)\n"
        "  high_time_pressure_ratio     (float, default: 1.3)\n\n"
        "Example:\n"
        "  ros2 run simulated_plan_component simulated_plan_component --ros-args "
        "-p battery_drainage_per_poi:=6.0 -p battery_safety_margin:=12.0 "
        "-p low_battery_threshold:=40.0 -p high_time_pressure_ratio:=1.25\n"
    )


def main(args=None):
    if args is None:
        args = sys.argv

    if '--help' in args or '-h' in args:
        usage()
        return

    rclpy.init(args=args)
    logger = rcutils_logger.RcutilsLogger(name="my_logger")
    logger.info("Starting Simulated Plan Component...")

    with Executor() as executor: 

        simulated_plan_component = SimulatedPlanComponent()
        rclpy.spin(simulated_plan_component)


if __name__ == '__main__':
    main(sys.argv)