#!/usr/bin/env python

# begin imports
import json
import yaml
import websocket
import sys
import rclpy
import rosidl_runtime_py
from rclpy.node import Node
from threading import *
from rosmonitoring_interfaces.msg import MonitorError
from std_msgs.msg import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import *
from notify_user_interfaces.srv import *
# done import

class ROSMonitor_monitor_prop1(Node):


	def callback_battery_status(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='/battery_status'
		dict['time']=float(self.get_clock().now().to_msg().sec) + float((self.get_clock().now().to_msg().nanosec) / 1000000000)
		self.ws_lock.acquire()
		while dict['time'] in self.dict_msgs:
			dict['time']+=0.01
		self.ws.send(json.dumps(dict))
		self.dict_msgs[dict['time']] = data
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		self.on_message_topic(message)

	def callback_NotifyUserComponent_StartAlarm__service_event(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='/NotifyUserComponent/StartAlarm/_service_event'
		dict['time']=float(self.get_clock().now().to_msg().sec) + float((self.get_clock().now().to_msg().nanosec) / 1000000000)
		self.ws_lock.acquire()
		while dict['time'] in self.dict_msgs:
			dict['time']+=0.01
		self.ws.send(json.dumps(dict))
		self.dict_msgs[dict['time']] = data
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		self.on_message_topic(message)

	def __init__(self,monitor_name,log,actions):
		self.monitor_publishers={}
		self.config_publishers={}
		self.config_subscribers={}
		self.config_client_services={}
		self.config_server_services={}
		self.services_info={}
		self.dict_msgs={}
		self.ws_lock=Lock()
		self.name=monitor_name
		self.actions=actions
		self.logfn=log
		self.topics_info={}
		super().__init__(self.name)
		# creating the verdict and error publishers for the monitor
		self.monitor_publishers['error']=self.create_publisher(topic=self.name+'/monitor_error',msg_type=MonitorError,qos_profile=1000)

		self.monitor_publishers['verdict']=self.create_publisher(topic=self.name+'/monitor_verdict',msg_type=String,qos_profile=1000)

		# done creating monitor publishers

		self.publish_topics=False
		self.topics_info['/battery_status']={'package': 'sensor_msgs.msg', 'type': 'BatteryState'}
		self.topics_info['/NotifyUserComponent/StartAlarm/_service_event']={'package': 'notify_user_interfaces.srv', 'type': 'StartAlarm_Event'}
		self.config_subscribers['/battery_status']=self.create_subscription(topic='/battery_status',msg_type=BatteryState,callback=self.callback_battery_status,qos_profile=1000)

		self.config_subscribers['/NotifyUserComponent/StartAlarm/_service_event']=self.create_subscription(topic='/NotifyUserComponent/StartAlarm/_service_event',msg_type=StartAlarm_Event,callback=self.callback_NotifyUserComponent_StartAlarm__service_event,qos_profile=1000)

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )
		websocket.enableTrace(True)
		self.ws = websocket.WebSocket()
		self.ws.connect('ws://127.0.0.1:8081')
		self.get_logger().info('Websocket is open')


	def on_message_topic(self,message):
		json_dict = json.loads(message)
		verdict = str(json_dict['verdict'])
		if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
			if verdict == 'true' and not self.publish_topics:
				self.get_logger().info('The monitor concluded the satisfaction of the property under analysis and can be safely removed.')
				self.ws.close()
				exit(0)
			else:
				self.logging(json_dict)
				topic = json_dict['topic']
				self.get_logger().info('The event '+message+' is consistent and republished')
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(self.dict_msgs[json_dict['time']])
				del self.dict_msgs[json_dict['time']]
		else:
			self.logging(json_dict)
			self.get_logger().info('The event' + message + ' is inconsistent' )
			error = MonitorError()
			error.m_topic = json_dict['topic']
			error.m_time = json_dict['time']
			error.m_property = json_dict['spec']
			error.m_content = str(self.dict_msgs[json_dict['time']])
			self.monitor_publishers['error'].publish(error)
			if verdict == 'false' and not self.publish_topics:
				self.get_logger().info('The monitor concluded the violation of the property under analysis and can be safely removed.')
				self.ws.close()
				exit(0)
			if self.actions[json_dict['topic']][0] != 'filter':
				topic = json_dict['topic']
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(self.dict_msgs[json_dict['time']])
				del self.dict_msgs[json_dict['time']]
			error=True
		verdict_msg = String()
		verdict_msg.data = verdict
		self.monitor_publishers['verdict'].publish(verdict_msg)

	def logging(self,json_dict):
		try:
			with open(self.logfn,'a+') as log_file:
				log_file.write(json.dumps(json_dict)+'\n')
			self.get_logger().info('Event logged')
		except:
			self.get_logger().info('Unable to log the event')

def main(args=None):
	rclpy.init(args=args)
	log = './log.txt'
	actions = {}
	actions['/battery_status']=('log',0)
	actions['/NotifyUserComponent/StartAlarm/_service_event']=('log',0)
	monitor = ROSMonitor_monitor_prop1('monitor_prop1',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
