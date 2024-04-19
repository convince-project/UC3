import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from scheduler_interfaces.srv import GetCurrentAction, UpdateAction
from text_to_speech_interfaces.srv import Speak

class MyService(Node):

    def __init__(self):
        super().__init__('NarrateComponentNode')
        self.client = self.create_client(GetCurrentAction, '/SchedulerComponent/GetCurrentAction')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.client = self.create_client(UpdateAction, '/SchedulerComponent/UpdateAction')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.client = self.create_client(Speak, '/TextToSpeechComponent/Speak')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.service = self.create_service(Trigger, '/NarrateComponent/Narrate', self.callback)

    def callback(self, request, response):
        done_with_poi = False
        while not done_with_poi:
            # call the GetCurrentAction service
            request_getCurrentAction = GetCurrentAction.Request()
            future_getCurrentAction = self.client.call_async(request_getCurrentAction)
            rclpy.spin_until_future_complete(self, future_getCurrentAction)
            if future_getCurrentAction.result() is not None:
                response.text = future_getCurrentAction.result().action
                self.get_logger().info('Response: {0}'.format(response.text))

                # call the text-to-speech service
                request_speak = Speak.Request()
                request_speak.text = response.text
                future_speak = self.client.call_async(request_speak)
                rclpy.spin_until_future_complete(self, future_speak)
                if future_speak.result() is not None:
                    response.text = future_speak.result().text
                    self.get_logger().info('Response: {0}'.format(response.text))

            #call the UpdateAction service
            request_updateAction = UpdateAction.Request()
            future_updateAction = self.client.call_async(request_updateAction)
            rclpy.spin_until_future_complete(self, future_updateAction)
            if future_updateAction.result() is not None:
                response.text = future_updateAction.result().action
                self.get_logger().info('Response: {0}'.format(response.text))         
                done_with_poi = future_updateAction.result().done_with_poi
        response.success = True
        return response
    

    
def main(args=None):
    rclpy.init(args=args)
    node = MyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
