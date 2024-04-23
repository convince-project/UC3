import rclpy
from rclpy.node import Node
from narrate_interfaces.srv import Narrate, IsDone
from scheduler_interfaces.srv import GetCurrentAction, UpdateAction
from text_to_speech_interfaces.srv import Speak
from text_to_speech_interfaces.msg import DoneSpeaking
import threading

global done

class NodeExecutor(Node):
    is_done_speaking = False
    def __init__(self):
        super().__init__(node_name = 'NarrateComponentThreadNode')

        self.client = self.create_client(GetCurrentAction, '/SchedulerComponent/GetCurrentAction')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.client = self.create_client(UpdateAction, '/SchedulerComponent/UpdateAction')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # self.client = self.create_client(Speak, '/TextToSpeechComponent/Speak')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')

        # create a callback on /TextToSpeechComponent/IsDone topic
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10) 
        


    def listener_callback(self, msg):
        self.is_done_speaking = msg.is_done


    def run():
        done_with_poi = False
        while not done_with_poi:
            # call the GetCurrentAction service
            request_getCurrentAction = GetCurrentAction.Request()
            future_getCurrentAction = self.client.call_async(request_getCurrentAction)
            rclpy.spin_until_future_complete(self, future_getCurrentAction)
            if future_getCurrentAction.result() is not None:
                text_to_send = future_getCurrentAction.result().action
                self.get_logger().info('Response: {0}'.format(text_to_send))

                # # call the text-to-speech service
                # request_speak = Speak.Request()
                # request_speak.text = text_to_send
                # future_speak = self.client.call_async(request_speak)
                # rclpy.spin_until_future_complete(self, future_speak)
                # if future_speak.result() is not None:
                #     if future_speak.result().is_ok:
                #         self.is_done_speaking = False
                #         while not self.is_done_speaking:
                #             self.sleep(0.15)

            #call the UpdateAction service
            request_updateAction = UpdateAction.Request()
            future_updateAction = self.client.call_async(request_updateAction)
            rclpy.spin_until_future_complete(self, future_updateAction)
            if future_updateAction.result() is not None:       
                done_with_poi = future_updateAction.result().done_with_poi
        done = True




class MyService(Node):

    def __init__(self):
        super().__init__('NarrateComponentNode')
        self.nodeExecutor = NodeExecutor()
        self.service = self.create_service(Narrate, '/NarrateComponent/Narrate', self.callback)
        self.service = self.create_service(IsDone, '/NarrateComponent/IsDone', self.callbackIsDone)

    def callback(self, request, response):
        # launch the thread
        done = False
        # launch NodeExecutor class with lambda function
        
        nodeExecutorInstance = threading.Thread(target=lambda: self.nodeExecutor.run())
        response.success = True
        return response
    
    def callbackIsDone(self, request, response):
        response.success = done
        return response


# create a thread class that executes the main function



def main(args=None):
    rclpy.init(args=args)
    node = MyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
