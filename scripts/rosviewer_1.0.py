#!/usr/bin/env python
from __future__ import division, print_function
from operator import itemgetter
from appJar import gui
import rospy
import genpy
import rosgraph
import roslib.message
import socket
import os
import sys
import subprocess
import time

NAME = 'rostopic'
echo = ''

#----------makelist----------#
def makelist(result):
	result_tmp = list()
	string = ""
	j = 0
	for i in range(result.count('\n')):
		while result[j] != '\n':
			string += result[j]
			j += 1
		result_tmp.append(string)
		string = ""
		j += 1
	result = result_tmp
	return result
#----------makelist----------#

#----------rostopic echo 'topic'----------#
class ROSTopicException(Exception):
	pass

class ROSTopicIOException(ROSTopicException):
	pass

class CallbackEcho(object):
    def __init__(self, topic, msg_eval, plot=False, filter_fn=None,
                 echo_clear=False, echo_all_topics=False,
                 offset_time=False, count=None,
                 field_filter_fn=None, fixed_numeric_width=None,
                 value_transform_fn=None):
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        self.msg_eval = msg_eval
        self.plot = plot
        self.filter_fn = filter_fn
        self.fixed_numeric_width = fixed_numeric_width

        self.prefix = ''
        self.suffix = '\n---' if not plot else ''# same as YAML document separator, bug #3291
        
        self.echo_all_topics = echo_all_topics
        self.offset_time = offset_time

        # done tracks when we've exceeded the count
        self.done = False
        self.max_count = count
        self.count = 0

        # determine which strifying function to use
        if plot:
            #TODOXXX: need to pass in filter function
            self.str_fn = _str_plot
            self.sep = ''
        else:
            #TODOXXX: need to pass in filter function
            self.str_fn = self.custom_strify_message
            if echo_clear:
                self.prefix = '\033[2J\033[;H'

        self.field_filter=field_filter_fn
        self.value_transform=value_transform_fn
        
        # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
        self.first = True

        # cache
        self.last_topic = None
        self.last_msg_eval = None

    def custom_strify_message(self, val, indent='', time_offset=None, current_time=None, field_filter=None,
                              type_information=None, fixed_numeric_width=None, value_transform=None):
        # ensure to print uint8[] as array of numbers instead of string
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        if value_transform is not None:
            val = value_transform(val, type_information)
        return genpy.message.strify_message(val, indent=indent, time_offset=time_offset, current_time=current_time, field_filter=field_filter, fixed_numeric_width=fixed_numeric_width)

    def callback(self, data, callback_args, current_time=None):
        topic = callback_args['topic']
        type_information = callback_args.get('type_information', None)
        if self.filter_fn is not None and not self.filter_fn(data):
            return

        if self.max_count is not None and self.count >= self.max_count:
            self.done = True
            return
        
        try:
            msg_eval = self.msg_eval
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                # self.topic is actually a reference to topic field, generate msgeval
                if topic == self.last_topic:
                    # use cached eval
                    msg_eval = self.last_msg_eval
                else:
                    # generate msg_eval and cache
                    self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
                    self.last_topic = topic
            elif not self.echo_all_topics:
                return

            if msg_eval is not None:
                data = msg_eval(data)
                
            # data can be None if msg_eval returns None
            if data is not None:
                # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
                
                self.count += 1
                
                # print fields header for plot
                if self.plot and self.first:
                    sys.stdout.write("%"+_str_plot_fields(data, 'field', self.field_filter)+'\n')
                    self.first = False

                if self.offset_time:
                    sys.stdout.write(self.prefix+\
                                     self.str_fn(data, time_offset=rospy.get_rostime(),
                                                 current_time=current_time, field_filter=self.field_filter,
                                                 type_information=type_information, fixed_numeric_width=self.fixed_numeric_width,
                                                 value_transform=self.value_transform) + \
                                     self.suffix + '\n')
                else:
                    global echo
                    echo = self.prefix+\
                                     self.str_fn(data,
                                                 current_time=current_time, field_filter=self.field_filter,
                                                 type_information=type_information, fixed_numeric_width=self.fixed_numeric_width,
                                                 value_transform=self.value_transform) + \
                                     self.suffix + '\n'
                    if(echo == ""):
                        print(echo)
                # we have to flush in order before piping to work
                sys.stdout.flush()
            # #2778 : have to chcallback_echoeck count after incr to set done flag
            if self.max_count is not None and self.count >= self.max_count:
                self.done = True

        except IOError:
            self.done = True
        except:
            # set done flag so we exit
            self.done = True
            traceback.print_exc()
            
def _check_master():
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Please execute roscore :D")
    
def get_topic_class(topic, blocking=False):
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % topic_type)
    return msg_class, real_topic, msg_eval

def get_topic_type(topic, blocking=False):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def _get_topic_type(topic):
    try:
        val = _master_get_topic_types(rosgraph.Master('/rostopic'))
    except socket.error:
        raise ROSTopicIOException("Please execute rosmaster :D")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)

        # try to ignore messages which don't have the field specified as part of the topic name
        while matches:
            t, t_type = matches[0]
            msg_class = roslib.message.get_message_class(t_type)
            if not msg_class:
                # if any class is not fetchable skip ignoring any message types
                break
            msg = msg_class()
            nested_attributes = topic[len(t) + 1:].rstrip('/')
            nested_attributes = nested_attributes.split('[')[0]
            if nested_attributes == '':
                break
            try:
                _get_nested_attribute(msg, nested_attributes)
            except AttributeError:
                # ignore this type since it does not have the requested field
                matches.pop(0)
                continue
            matches = [(t, t_type)]
            break

    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None
    
def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def msgevalgen(pattern):
    evals = []  # list of (field_name, slice_object) pairs
    fields = [f for f in pattern.split('/') if f]
    for f in fields:
        if '[' in f:
            field_name, rest = f.split('[', 1)
            if not rest.endswith(']'):
                print("missing closing ']' in slice spec '%s'" % f, file=sys.stderr)
                return None
            rest = rest[:-1]  # slice content, removing closing bracket
            try:
                array_index_or_slice_object = _get_array_index_or_slice_object(rest)
            except AssertionError as e:
                print("field '%s' has invalid slice argument '%s': %s"
                      % (field_name, rest, str(e)), file=sys.stderr)
                return None
            evals.append((field_name, array_index_or_slice_object))
        else:
            evals.append((f, None))

    def msgeval(msg, evals):
        for i, (field_name, slice_object) in enumerate(evals):
            try: # access field first
                msg = getattr(msg, field_name)
            except AttributeError:
                print("no field named %s in %s" % (field_name, pattern), file=sys.stderr)
                return None

            if slice_object is not None: # access slice
                try:
                    msg = msg.__getitem__(slice_object)
                except IndexError as e:
                    print("%s: %s" % (str(e), pattern), file=sys.stderr)
                    return None

                # if a list is returned here (i.e. not only a single element accessed),
                # we need to recursively call msg_eval() with the rest of evals
                # in order to handle nested slices
                if isinstance(msg, list):
                    rest = evals[i + 1:]
                    return [msgeval(m, rest) for m in msg]
        return msg

    return (lambda msg: msgeval(msg, evals)) if evals else None

def _sleep(duration):
		rospy.rostime.wallsleep(duration)

def _rostopic_echo(topic, callback_echo, bag_file=None, echo_all_topics=False):
    if bag_file:
        rospy.rostime.set_rostime_initialized(True)        
        _rostopic_echo_bag(callback_echo, bag_file)
    else:
        _check_master()
        rospy.init_node('rostopic', anonymous=True)
        msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
        if msg_class is None:
            return
        callback_echo.msg_eval = msg_eval
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information.split('[', 1)[0])
                        if not submsg_class:
                            raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        global sub
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})
        if use_sim_time:
            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                _sleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")
		_sleep(0.1)
		
#----------rostopic echo 'topic'----------#

#--------------------make app--------------------#
result = subprocess.check_output('rostopic list', shell = True)
topiclist = makelist(result)

subFlag = False
recordFlag = False
recordcnt = topiclist[:]
filelist = ''
bag_data = {}

app = gui("RosViewer","850x800")
app.setBg("light gray")
app.setLocation("CENTER")

location = os.getcwd()+'/bagging/'

for i in range(len(topiclist)):
	recordcnt[i] = 0
	

#----------TAP1----------#
def pressTopic(topicecho):
	global subFlag
	global recordtopic
	recordtopic = topicecho[1:]
	if subFlag is True:
		global sub
		sub.unregister()
	_rostopic_echo(topicecho, CallbackEcho(topicecho, None))
	app.setPollTime(100)
	app.registerEvent(setlabeltopic)
	subFlag = True

def Search():
	text = '#' + app.getEntry("e1")
	topicsearch = topiclist[:]
	for i in range(len(topiclist)):
		topicsearch[i] = '#' + topicsearch[i]
		if len(text) <= len(topicsearch[i]):
			for j in range(len(text)):
				if topicsearch[i][j] == text[j]:
					app.showButton(topiclist[i])
				else:
					app.hideButton(topiclist[i])
					break
		else:
			app.hideButton(topiclist[i])

def setlabeltopic():
	app.setLabel("topic", echo)

def pressrecord(Record):
	global recordFlag
	recordFlag = True

def record():
	global recordcnt, recordFlag, recordtopic
	real_record = ''
	if recordFlag == True:
		try:
			for i in range(len(recordtopic)):
				if recordtopic[i] == '/':
					real_record = ''
					continue
				real_record += recordtopic[i]
			
			filet = open(location + str(topiclist.index('/'+recordtopic))+" - "+real_record+".txt", 'a')
			filet.write("joonrecord"+str(recordcnt[topiclist.index('/'+recordtopic)])+'\n')
			print(str(recordcnt[topiclist.index('/'+recordtopic)]))
			filet.write(echo)
			filet.close()
			recordcnt[topiclist.index('/'+recordtopic)] += 1
			app.setButton("Record", "Recording...")
		except:
			app.warningBox("RecordError", "Please click topic first")
			recordFlag = False

def pressstop(Stop):
	global recordFlag
	recordFlag = False
	app.setButton("Record", "Record")
	
app.startTabbedFrame("TabbedFrame")
app.setTabbedFrameTabExpand("TabbedFrame", expand=True)
app.startTab("RosEchoTopic")
app.startPanedFrameVertical("p1")
app.setSticky("ew")
app.setPadding([140,10])
app.startFrame("searchTopic")
app.setInPadding([50,0])
app.addAutoEntry("e1", topiclist, 0, 0)
app.setEntryDefault("e1", "Please enter your search")
app.setInPadding([0,0])
app.setPadding([5,0])
app.addButton("Record", pressrecord, 0, 1)
app.addButton("Stop", pressstop, 0, 2)
app.stopFrame()

app.startPanedFrame("p2")
app.startScrollPane("s1",1,0,2)
for i in range(len(topiclist)):
	app.setPadding([10,10])
	app.addButton(topiclist[i], pressTopic)
	app.setButtonAlign(topiclist[i], "left")
	app.setButtonBg(topiclist[i], "white")
	app.setButtonFg(topiclist[i], "black")
app.stopScrollPane()

app.startPanedFrame("p3")
app.startScrollPane("s2")
app.addLabel("topic", "Please Click Topic")
app.stopScrollPane()
app.stopPanedFrame()

app.stopPanedFrame()
app.stopPanedFrame()

app.setPollTime(100)
app.registerEvent(Search)
app.registerEvent(record)
app.stopTab()
#----------TAP1----------#

#----------TAP2----------#
def Search():
	text = '#' + app.getEntry("e2")
	topicsearch = filelist[:]
	for i in range(len(topiclist)):
		topicsearch[i] = '#' + topicsearch[i]
		if len(text) <= len(topicsearch[i]):
			for j in range(len(text)):
				if topicsearch[i][j] == text[j]:
					app.showButton(topiclist[i])
				else:
					app.hideButton(topiclist[i])
					break
		else:
			app.hideButton(topiclist[i])
			
def pressrm(Remove):
	os.system("cd " + location + "&& rm -rf ./*")  
	for i in range(len(filelist)):
			app.hideButton(filelist[i])

def Refresh():
	global filelist
	filelist = os.listdir(location)
	try:
		for i in range(len(filelist)):
			app.showButton(filelist[i])
	except:
		print("Is there files that is not being in topiclist in bagging folder??")
		sys.exit()

def pressBagging(Bag):
	global filelist, bag_name
	bag_name = Bag
	linecnt = -1
	linedata = []
	fileb = open(location + Bag, 'r')
	line = "Made By JoonYeol"
	
	while line:
		line = fileb.readline()
		if line == 'joonrecord'+str(linecnt+1)+'\n':
			linedata.append('')
			linecnt += 1
		else:
			linedata[linecnt] += line
	fileb.close()
	bag_data[Bag] = linedata

	app.setScaleRange("time", 0, linecnt, curr=None)
	app.showScaleIntervals("time", linecnt/4)
	setlabelbag("time")
	app.setScaleChangeFunction("time", setlabelbag)

def setlabelbag(time):
	global bag_name
	num = app.getScale("time")
	app.setLabel("Bag",bag_data[bag_name][num])
	
app.startTab("RosEchoBagging")
app.startPanedFrameVertical("p4")

app.setSticky("ew")
app.setPadding([140,10])
app.startFrame("searchBagging")
app.setInPadding([50,0])
app.addAutoEntry("e2", filelist, 0, 0)
app.setEntryDefault("e2", "Please enter your search")
app.setInPadding([0,0])
app.setPadding([5,0])
app.addButton("Remove", pressrm, 0, 2)
app.addScale("time", 1, 0, 3)
app.showScaleIntervals("time", 25)
app.showScaleValue("time", show=True)
app.stopFrame()

app.startPanedFrame("p5")
app.startScrollPane("s3",1,0,2)
for i in range(len(topiclist)):
	bag = ''
	for j in range(len(topiclist[i])):
		if topiclist[i][j] == '/':
			bag = ''
			continue
		bag += topiclist[i][j]
	bag = str(i) +" - " + bag +".txt"
	app.setPadding([10,10])
	app.addButton(bag, pressBagging)
	app.setButtonAlign(bag, "left")
	app.setButtonBg(bag, "white")
	app.setButtonFg(bag, "black")
	app.hideButton(bag)
app.stopScrollPane()

app.startPanedFrame("p6")
app.startScrollPane("s4")
app.addLabel("Bag", "Please Click Bagfile")
app.stopScrollPane()
app.stopPanedFrame()
app.stopPanedFrame()

app.stopPanedFrame()
app.registerEvent(Refresh)
app.stopTab()
#----------TAP2----------#
app.stopTabbedFrame()

app.go()
#--------------------make app--------------------#
