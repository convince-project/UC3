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
# done import

class ROSMonitor_monitor_prop8(Node):


	def callback_joint_states_controlModes(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='/joint_states/controlModes'
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
		self.topics_info['/joint_states/controlModes']={'package': 'std_msgs.msg', 'type': 'Int8MultiArray'}
		self.config_subscribers['/joint_states/controlModes']=self.create_subscription(topic='/joint_states/controlModes',msg_type=Int8MultiArray,callback=self.callback_joint_states_controlModes,qos_profile=1000)

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )
		websocket.enableTrace(True)
		self.ws = websocket.WebSocket()
		self.ws.connect('ws://127.0.0.1:8088')
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
	actions['/joint_states/controlModes']=('log',0)
	monitor = ROSMonitor_monitor_prop8('monitor_prop8',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
