import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dialog_interfaces.srv import Answer_Event, InterpretCommand_Event, ShortenReply_Event


class DialogServiceEventForwarder(Node):
    def __init__(self):
        super().__init__('dialog_service_event_forwarder')

        # publisher for the aggregated event content
        self.publisher = self.create_publisher(String, '/DialogComponent/ServiceEvents', 10)

        # subscribe to the service event topics
        self.sub_answer = self.create_subscription(
            Answer_Event,
            '/DialogComponent/Answer/_service_event',
            self._on_answer_event,
            10,
        )
        self.sub_interpret = self.create_subscription(
            InterpretCommand_Event,
            '/DialogComponent/InterpretCommand/_service_event',
            self._on_interpret_command_event,
            10,
        )
        self.sub_shorten = self.create_subscription(
            ShortenReply_Event,
            '/DialogComponent/ShortenReply/_service_event',
            self._on_shorten_reply_event,
            10,
        )

        self.get_logger().info('DialogServiceEventForwarder initialized and subscribed to service event topics.')

    def _on_answer_event(self, msg: Answer_Event):
        text = self._event_to_text('Answer', msg)
        self._publish(text)

    def _on_interpret_command_event(self, msg: InterpretCommand_Event):
        text = self._event_to_text('InterpretCommand', msg)
        self._publish(text)

    def _on_shorten_reply_event(self, msg: ShortenReply_Event):
        text = self._event_to_text('ShortenReply', msg)
        self._publish(text)

    def _event_to_text(self, service_name: str, event_msg):
        event_dict = {
            'service': service_name,
            'info': {
                'stamp_sec': int(event_msg.info.stamp.sec),
                'stamp_nanosec': int(event_msg.info.stamp.nanosec),
                #'publisher_gid': list(event_msg.info.publisher_gid),
            },
            'requests': [],
            'responses': [],
        }

        for req in event_msg.request:
            request_data = {}
            for key, val in req._fields_and_field_types.items():
                request_data[key] = getattr(req, key)
            event_dict['requests'].append(request_data)

        for resp in event_msg.response:
            response_data = {}
            for key, val in resp._fields_and_field_types.items():
                response_data[key] = getattr(resp, key)
            event_dict['responses'].append(response_data)

        return json.dumps(event_dict, ensure_ascii=False)

    def _publish(self, payload: str):
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published aggregated service event: {payload}')


def main(args=None):
    rclpy.init(args=args)
    node = DialogServiceEventForwarder()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
