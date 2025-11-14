import time
from rclpy.impl import rcutils_logger

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from congestion_coverage_plan_museum.mdp.MDP import MDP, State
from congestion_coverage_plan_museum.map_utils.OccupancyMap import OccupancyMap
from congestion_coverage_plan_museum.cliff_predictor.PredictorCreator import create_generic_cliff_predictor
from congestion_coverage_plan_museum.bt_utils.BTWriter import BTWriter
from planner_component.DetectionsRetriever import DetectionsRetriever
from congestion_coverage_plan_museum.solver.LrtdpTvmaAlgorithm import LrtdpTvmaAlgorithm
from planner_interfaces.action import Plan
from blackboard_interfaces.srv import GetIntBlackboard
import sys

class PlannerComponent(Node):

    def __init__(self, 
                 occupancy_map_path,
                 cliff_map_path,
                 time_bound_lrtdp,
                 time_bound_real, 
                 convergence_threshold,
                 wait_time, 
                 explain_time,
                 detections_topic="static_tracks", 
                 bt_file_path="src/behavior_tree/BT/bt_scheduler.xml"):
        super().__init__('planner_component')
        self._action_server = ActionServer(
            self,
            Plan,
            '/PlannerComponent/Plan',
            self.execute_callback)
        self.get_logger().info('Planner Component has been started.')
        self._occupancy_map_path = occupancy_map_path
        self._cliff_map_path = cliff_map_path
        self._time_bound_lrtdp = time_bound_lrtdp
        self._time_bound_real = time_bound_real
        self._convergence_threshold = convergence_threshold
        self._wait_time = wait_time
        self._explain_time = explain_time
        self._started = False
        self._start_time = None
        self._start_vertex = "vertex1"
        self._doors = ["vertex13", "vertex14", "vertex15", "vertex16"]
        self._final_vertex = "vertex12"
        self._visited_vertices = []
        self._pois_explained = []
        self._time_for_occupancies = None
        self._btWriter = BTWriter(bt_file_path)

        self.client = self.create_client(GetIntBlackboard, 'BlackboardComponent/GetInt')
        num_retries = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /BlackboardComponent/GetInt not available, waiting again...')
            num_retries = num_retries + 1
            if num_retries > 10:
                self.get_logger().error('Service /BlackboardComponent/GetInt not available, exiting...')
                sys.exit(1)

        self._detections_retriever = DetectionsRetriever(self, detections_topic)


    def retrieve_blackboard_value(self, key):
        request = GetIntBlackboard.Request()
        request.field_name = key
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().value
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))
            return None


    def compute_current_state(self):
        pois_done = []
        poidone0 = self.retrieve_blackboard_value('PoiDone0')
        self._visited_vertices = []
        if self._start_time is None:
            self._start_time = self.get_clock().now().seconds_nanoseconds()[0]

        if poidone0 == None or poidone0 == 0:
            self._start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self._time_for_occupancies = self.get_clock().now().seconds_nanoseconds()[0]
            return State(self._start_vertex, 
                                           0,
                                           set([self._start_vertex]),
                                           set())
        else:
            for i in range(1, 12):
                key = f'PoiDone{i}'
                vertex_done = self.retrieve_blackboard_value(key-1)
                if vertex_done is not None and vertex_done == 1:
                    pois_done.append(int(i/2))
                    self._visited_vertices.append("vertex" + str(i))
            # last_poi = pois_done[-1]
            last_vertex = self._visited_vertices[-1]
            if len(pois_done) > 1:
                for i in range(0, len(pois_done)-1):
                    self._visited_vertices.append(self._doors[i])
            self._time_for_occupancies = self.get_clock().now().seconds_nanoseconds()[0]
            return State(last_vertex, 
                        self.get_clock().now().seconds_nanoseconds()[0] - self._start_time,
                        set([self._start_vertex] + [vertex for vertex in self._visited_vertices]),
                        set(poi_done for poi_done in pois_done))



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # get the current status
        predictor = create_generic_cliff_predictor(self._cliff_map_path)
        print("detections retriever in planner component:", self._detections_retriever)

        occupancy_map = OccupancyMap(cliffPredictor=predictor, detections_retriever=self._detections_retriever)
        occupancy_map.load_occupancy_map(self._occupancy_map_path)
        current_state = self.compute_current_state()
        lrtdp = LrtdpTvmaAlgorithm(occupancy_map=occupancy_map,
                            initial_state_name=current_state,
                            convergence_threshold=self._convergence_threshold,
                            time_bound_real=self._time_bound_real,
                            planner_time_bound=self._time_bound_lrtdp,
                            time_for_occupancies=self._time_for_occupancies,
                            time_start=self._start_time,
                            wait_time=self._wait_time,
                            explain_time=self._explain_time,
                            heuristic_function="madama_experiments",
                            initial_state=current_state)
        result = lrtdp.solve()
        policy = lrtdp.policy
        if result is None:
            self.get_logger().error('No plan found.')
            result = Plan.Result()
            result.is_ok = False
            try:
                goal_handle.abort()
            except Exception as e:
                self.get_logger().error(f'Error while aborting goal: {e}')
            return result
        # iterate over the policy to extract the plan
        if str(current_state) in policy:
            action = policy[str(current_state)]
            print(action)
            # poi_number = action.replace("vertex", "")
            # if action in range(1, 11):
            #     pois_list.append(poi_number)
        plan = self.iterate_over_policy(policy, current_state)
        # plan = [2,4,5,7,9]
        plan = [int(poi) - 1  for poi in plan]
        self._btWriter.recreateBTWithPlan(plan)
        self._btWriter.write()
        try:
            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f'Error while succeeding goal: {e}')

        self.get_logger().info(f"sequence_of_pois: {plan}")
        result = Plan.Result()
        result.is_ok = True
        # result.sequence = feedback_msg.partial_sequence
        return result


    def iterate_over_policy(self, policy, current_state):
        state = current_state
        # plan_sequence = []
        sequence_of_pois = []
        print(policy[str(state)])
        print("state:", policy[str(state)][1].to_string())
        print(len(policy))
        while state is not None:
            action = policy[str(state)][2] # get action
            if action == "explain":
                sequence_of_pois.append(policy[str(state)][1].get_vertex().replace("vertex", ""))
            state = policy[str(state)][3] # get action
            
            # next_state = self._mdp.get_next_state(state, action)
        #     if action == "explain":
        #         sequence_of_pois.append(policy[str(state)][1].get_vertex())
        #     state = policy[str(state)][1] # get next state
        #     self.get_logger().info(f"Next state: {state}")
        self.get_logger().info(f"sequence_of_pois: {sequence_of_pois}")
        return sequence_of_pois


def main(args=None):
    ## parse args
    rclpy.init(args=args)
    logger = rcutils_logger.RcutilsLogger(name="my_logger")
    logger.info("Starting Planner Component...")
    print(sys.argv)
    print("Starting Planner Component...")
    if "--occupancy_map_path" in sys.argv and len(sys.argv) >= sys.argv.index("--occupancy_map_path") + 1:
        occupancy_map_path = sys.argv[sys.argv.index("--occupancy_map_path") + 1]
    else:
        sys.exit("Error: occupancy_map_path argument not provided")
    if "--cliff_map_path" in sys.argv and len(sys.argv) >= sys.argv.index("--cliff_map_path") + 1:
        cliff_map_path = sys.argv[sys.argv.index("--cliff_map_path") + 1]
    else:
        sys.exit("Error: cliff_map_path argument not provided")
    if "--time_bound_lrtdp" in sys.argv and len(sys.argv) >= sys.argv.index("--time_bound_lrtdp") + 1:
        time_bound_lrtdp = float(sys.argv[sys.argv.index("--time_bound_lrtdp") + 1])
    else:
        sys.exit("Error: time_bound_lrtdp argument not provided")
    if "--time_bound_real" in sys.argv and len(sys.argv) >= sys.argv.index("--time_bound_real") + 1:
        time_bound_real = float(sys.argv[sys.argv.index("--time_bound_real") + 1])
    else:
        sys.exit("Error: time_bound_real argument not provided")
    if "--convergence_threshold" in sys.argv and len(sys.argv) >= sys.argv.index("--convergence_threshold") + 1:
        convergence_threshold = float(sys.argv[sys.argv.index("--convergence_threshold") + 1])
    else:
        sys.exit("Error: convergence_threshold argument not provided")
    if "--wait_time" in sys.argv and len(sys.argv) >= sys.argv.index("--wait_time") + 1:
        wait_time = float(sys.argv[sys.argv.index("--wait_time") + 1])
    else:
        sys.exit("Error: wait_time argument not provided")
    if "--explain_time" in sys.argv and len(sys.argv) >= sys.argv.index("--explain_time") + 1:
        explain_time = float(sys.argv[sys.argv.index("--explain_time") + 1])
    else:
        sys.exit("Error: explain_time argument not provided")
    if "--bt_file_path" in sys.argv and len(sys.argv) >= sys.argv.index("--bt_file_path") + 1:
        bt_file_path = sys.argv[sys.argv.index("--bt_file_path") + 1]
    else:
        sys.exit("Error: bt_file_path argument not provided")
    if "--detections_topic" in sys.argv and len(sys.argv) >= sys.argv.index("--detections_topic") + 1:
        detections_topic = sys.argv[sys.argv.index("--detections_topic") + 1]
    else:
        sys.exit("Error: detections_topic argument not provided")
    print("Starting Planner Component...")
    print("arguments parsed successfully.")
    print("occupancy_map_path:", occupancy_map_path)
    print("cliff_map_path:", cliff_map_path)
    print("time_bound_lrtdp:", time_bound_lrtdp)
    print("time_bound_real:", time_bound_real)
    print("convergence_threshold:", convergence_threshold)
    print("wait_time:", wait_time)
    print("explain_time:", explain_time)
    print("bt_file_path:", bt_file_path)
    print("detections_topic:", detections_topic)

    planner_component = PlannerComponent(occupancy_map_path=occupancy_map_path,
                                         cliff_map_path=cliff_map_path,
                                         time_bound_lrtdp=time_bound_lrtdp,
                                         time_bound_real=time_bound_real,
                                         convergence_threshold=convergence_threshold,
                                         wait_time=wait_time,
                                         explain_time=explain_time,
                                         bt_file_path=bt_file_path,
                                         detections_topic=detections_topic
                                        )
    rclpy.spin(planner_component)


if __name__ == '__main__':
    main(sys.argv)