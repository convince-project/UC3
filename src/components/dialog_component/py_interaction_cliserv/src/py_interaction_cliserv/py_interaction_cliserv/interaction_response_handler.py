import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dialog_interfaces.msg import VerbalInteraction
import json


class InteractionResponseHandler(Node):
    def __init__(self):
        super().__init__('interaction_response_handler')

        # Publisher for responses
        self.response_publisher = self.create_publisher(String, '/DialogComponent/InteractionResponse', 10)

        # Subscribe to interaction messages
        self.sub_interaction = self.create_subscription(
            VerbalInteraction,
            '/DialogComponent/interaction',
            self._on_interaction_received,
            10,
        )

        # Subscribe to service events for responses
        self.sub_service_events = self.create_subscription(
            String,
            '/DialogComponent/ServiceEvents',
            self._on_service_event,
            10,
        )
        self.interaction = False
        self.last_service_event = None
        self.get_logger().info('InteractionResponseHandler initialized and subscribed to interaction and service event topics.')

    def _on_interaction_received(self, msg: VerbalInteraction):
        """Handle incoming verbal interaction messages."""
        self._publish_response(f'Interaction received: {msg.text}', start_separator=True)
        self.interaction = True
        if not msg.text or msg.text.strip() == '':
            # Empty string, publish "No Response"
            self.interaction = False
            self._publish_response('No Response, VAD or Speech-to-Text might have returned an empty string.', end_separator=True)

    def _on_service_event(self, msg: String):
        """Handle incoming service event messages."""
        try:
            # Parse the JSON data from the service event message
            event_data = json.loads(msg.data)

            # Check if 'responses' exists and is a list
            if 'responses' in event_data and len(event_data['responses']) > 0 and self.interaction:
                all_replies = []
                
                # Loop through each response in the 'responses' array
                for response in event_data['responses']:
                    reply = response.get('reply', [])
                    if reply:
                        # Append the reply to the list with a newline
                        all_replies.append(" ".join(reply))  # This ensures that each reply is on a new line
                
                if all_replies:
                    # Join all replies together with a newline between them
                    full_reply = " ".join(all_replies)  # Join all replies with newlines between them
                    self._publish_response(f'Reply received: {full_reply}', end_separator=True)
                    self.interaction = False  # Reset interaction flag after processing the reply
                else:
                    self.get_logger().warn(f'No "reply" found in the service event response.')

            else:
                self.get_logger().warn(f'No responses found in the service event message.')

        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode JSON from service event: {msg.data}')
        except KeyError as e:
            self.get_logger().error(f'Missing expected key in service event data: {e}')

    def _publish_response(self, payload: str, start_separator: bool = False, end_separator: bool = False):
        """Publish response message."""
        msg = String()
        msg.data = payload
        separator = "#" * 30
        msg_separator = String()
        msg_separator.data = separator
        if start_separator:
            self.response_publisher.publish(msg_separator)
        self.response_publisher.publish(msg)
        if end_separator:
            self.response_publisher.publish(msg_separator)
        self.get_logger().info(f'Published response: {payload}')


def main(args=None):
    rclpy.init(args=args)
    node = InteractionResponseHandler()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()