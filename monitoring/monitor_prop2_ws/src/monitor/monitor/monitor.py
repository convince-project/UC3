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

class ROSMonitor_monitor(Node):


	def callbackchatter(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='chatter'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.ws.send(json.dumps(dict))
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		self.on_message_topic(message)

	def callbacknumbers(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='numbers'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.ws.send(json.dumps(dict))
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

		self.config_publishers['chatter']=self.create_publisher(topic='chatter',msg_type=String,qos_profile=1000)

		self.config_publishers['numbers']=self.create_publisher(topic='numbers',msg_type=Int32,qos_profile=1000)

		self.publish_topics=True
		self.topics_info['chatter']={'package': 'std_msgs.msg', 'type': 'String'}
		self.topics_info['numbers']={'package': 'std_msgs.msg', 'type': 'Int32'}
		self.config_subscribers['chatter']=self.create_subscription(topic='chatter_mon',msg_type=String,callback=self.callbackchatter,qos_profile=1000)

		self.config_subscribers['numbers']=self.create_subscription(topic='numbers_mon',msg_type=Int32,callback=self.callbacknumbers,qos_profile=1000)

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )
		websocket.enableTrace(True)
		self.ws = websocket.WebSocket()
		self.ws.connect('ws://127.0.0.1:8080')
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
				del json_dict['topic']
				del json_dict['time']
				if 'verdict' in json_dict: del json_dict['verdict']
				ROS_message = eval(''+self.topics_info[topic]['type']+'()')
				rosidl_runtime_py.set_message_fields(ROS_message,json_dict)
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(ROS_message)
		else:
			self.logging(json_dict)
			self.get_logger().info('The event' + message + ' is inconsistent' )
			error = MonitorError()
			error.m_topic = json_dict['topic']
			error.m_time = json_dict['time']
			error.m_property = json_dict['spec']
			json_dict_copy = json_dict.copy()
			del json_dict_copy['topic']
			del json_dict_copy['time']
			del json_dict_copy['spec']
			error.m_content = json.dumps(json_dict_copy)
			self.monitor_publishers['error'].publish(error)
			if verdict == 'false' and not self.publish_topics:
				self.get_logger().info('The monitor concluded the violation of the property under analysis and can be safely removed.')
				self.ws.close()
				exit(0)
			if self.actions[json_dict['topic']][0] != 'filter':
				topic = json_dict['topic']
				del json_dict['topic']
				del json_dict['time']
				del json_dict['spec']
				if 'verdict' in json_dict: del json_dict['verdict']
				ROS_message = eval(''+self.topics_info[topic]['type']+'()')
				rosidl_runtime_py.set_message_fields(ROS_message,json_dict)
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(ROS_message)
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
	actions['chatter']=('filter',0)
	actions['numbers']=('filter',0)
	monitor = ROSMonitor_monitor('monitor',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
