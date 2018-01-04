#!/usr/bin/env python
from __future__ import print_function
from os.path import exists
from os import remove
import rospy
import genpy
import rosgraph
import roslib.message
import os
import sys
import time

class CallbackEcho(object):
    def __init__(self, topic, offset_time=False, fixed_numeric_width=None, value_transform_fn=None, field_filter_fn=None):
        self.topic = topic
        self.fixed_numeric_width = fixed_numeric_width
        self.suffix = '\n---'
        self.str_fn = self.custom_strify_message
        self.field_filter=field_filter_fn
        self.value_transform=value_transform_fn

    def custom_strify_message(self, val, indent='', time_offset=None, current_time=None, field_filter=None,
                              type_information=None, fixed_numeric_width=None, value_transform=None):
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        if value_transform is not None:
            val = value_transform(val, type_information)
        return genpy.message.strify_message(val, indent=indent, time_offset=time_offset, current_time=current_time, field_filter=field_filter, fixed_numeric_width=fixed_numeric_width)

    def callback(self, data, callback_args, current_time=None):
		topic = callback_args['topic']
		type_information = callback_args.get('type_information', None)
		echo = self.str_fn(data,current_time=current_time,	field_filter=self.field_filter,type_information=type_information,	fixed_numeric_width=self.fixed_numeric_width,value_transform=self.value_transform) + self.suffix + '\n'
		x = ''; y = ''; seq = ''
		for i in range(len(echo)):
			if echo[i] == 's' and echo[i+1] == 'e' and echo[i+2] == 'q':
				while echo[i+5] != '\n': 
					seq = seq + echo[i+5]; i += 1
			if echo[i] == 'x' and echo[i+1] == ':':
				while echo[i] != '\n': 
					x = x + echo[i]; i += 1
				i += 3
				while echo[i] != '\n':
					y = y + echo[i]; i += 1
				break
        
		global Input
		location = "/home/yujin/semantics/" + Input + ".yaml"
		startp = "locations:\n"
		with open(location, 'r+') as sefile:
			lines = sefile.readlines()
			sefile.seek(0)
			sefile.truncate()
			for line in lines:
				if line.startswith(startp):
					line = line.rstrip("\n") + "\n  Point%s:\n    name: 'sample'\n    description: 'sample description'\n    world: 'sample'\n    pose: { %s, %s, theta: 0.00}\n"%(seq,x,y)
				sefile.write(line)

def _rostopic_echo(topic, callback_echo):
    rospy.init_node('rostopic', anonymous=True)
    val = rosgraph.Master('/rostopic').getTopicTypes()
    matches = [(t, t_type) for t, t_type in val if t == topic]
    msg_class = roslib.message.get_message_class(matches[0][1])
    type_information = None
    sub = rospy.Subscriber(matches[0][0], msg_class, callback_echo.callback,{'topic': topic, 'type_information': type_information})
    rospy.spin()

Input = raw_input("Please write yaml name : ")
topicName = '/clicked_point'
_rostopic_echo(topicName, CallbackEcho(topicName))
