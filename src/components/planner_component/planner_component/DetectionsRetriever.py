# topic subrscriber for retrieving detections from a ROS topic
# it keeps the latest detection messages in a member variable

from threading import Lock
from static_devices_msgs.msg import DetectionsArray, SingleDetection
from matplotlib import pyplot as plt

class Detection:
    def __init__(self, person_id, positionx, positiony, timestamp, vx, vy):
        self.person_id = person_id
        self.positionx = positionx
        self.positiony = positiony
        self.timestamp = timestamp
        self.vx = vx
        self.vy = vy


class DetectionsRetriever:
    def __init__(self, node=None, topic_name="static_tracks", queue_size=10):
        """
        If `node` (rclpy.node.Node) is provided, subscription is created immediately.
        Otherwise call start(node=...) or start_background_node() to run standalone.
        """
        self._node = node
        self._topic_name = topic_name
        self._lock = Lock()
        self._detections = {}
        self._current_occupancies = {}
        self._queue_size = queue_size
        self._subscriber = None

        self.start_with_node(node)
        # plt.ion()
        # # set background image
        # self.fig_size = [-21.2,36.4, -53.4, 9.2 ]
        # self.fig, self.ax = plt.subplots()
        # self.img = plt.imread("/home/ste/ws/birmingham/congestion-aware-planning/data/maps/madama3.jpg")
        # self.ax.imshow(self.img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
    
        # # create a graph to be updated dynamically with the detections
        # # self.scatter = self.ax.scatter([], [])
        # self.ax.set_xlim(-10, 10)
        # self.ax.set_ylim(-10, 10)
        # self.ax.set_title('Detections')
        # self.ax.set_xlabel('X Position')
        # self.ax.set_ylabel('Y Position')

        


    def _callback(self, msg):
        detections_local = {}
        current_occupancies_local = []
        for detection in msg.detections:
            detection: SingleDetection
            detection_obj = Detection(
                person_id=detection.id,
                positionx=detection.x,
                positiony=detection.y,
                timestamp=detection.ts,
                vx=detection.vx,
                vy=detection.vy
            )
            existing = self._detections.get(detection.id, [])
            # keep newest first, limit to queue_size
            combined = [detection_obj] + existing[:5] if existing else [detection_obj]
            detections_local[detection.id] = sorted(combined, key=lambda x: x.timestamp, reverse=True)
            
            current_occupancies_local.append(Detection(
                person_id=detection.id,
                positionx=detection.x,
                positiony=detection.y,
                timestamp=detection.ts,
                vx=detection.vx,
                vy=detection.vy
            ))
        # print(f"DetectionsRetriever: received {len(msg.detections)} detections.")
        # print(detections_local)
        # plt.cla()
        # self.ax.imshow(self.img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
        # for det_id in detections_local:
        #     for det in detections_local[det_id]:
        #         plt.plot(det.positionx, det.positiony, 'o', label=f'ID {det.person_id}')
        # plt.draw()
        # plt.pause(0.01)
        with self._lock:
            self._detections = detections_local
            self._current_occupancies = current_occupancies_local

    def get_detections(self):
        with self._lock:
            return self._detections

    def get_current_occupancies(self):
        with self._lock:
            return self._current_occupancies

    def start_with_node(self, node):
        """Attach to an existing node (create subscription if needed)."""
        self._node = node
        if self._subscriber is None:
            self._subscriber = self._node.create_subscription(DetectionsArray, self._topic_name, self._callback, self._queue_size)
        return True

    # def start_background_node(self, node_name='detections_retriever_node'):
    #     """Create a private rclpy node and spin it in a background thread."""
    #     # initialize rclpy if not already
    #     try:
    #         if not rclpy.ok():
    #             rclpy.init()
    #     except Exception:
    #         # rclpy.ok() may raise if rclpy not initialized in some versions; ensure init
    #         try:
    #             rclpy.init()
    #         except Exception:
    #             pass

    #     # create node and subscription
    #     if self._node is None:
    #         self._node = rclpy.create_node(node_name)
    #     if self._subscriber is None:
    #         self._subscriber = self._node.create_subscription(DetectionsArray, self._topic_name, self._callback, self._queue_size)

    #     # spin in background thread
    #     thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
    #     thread.start()
    #     self._node.get_logger().info(f"DetectionsRetriever started, listening to: {self._topic_name}")
    #     return True
