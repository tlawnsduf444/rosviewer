#!/usr/bin/env python
import rospy
from rosecho import rosecho
from rosecho import roslist
from appJar import gui
import os
import sys

echo = ''
topiclist = roslist.makelist()
subFlag = False
recordFlag = False
recordcnt = list()
baglist = topiclist[:]
filelist = ''
bag_data = {}

app = gui("RosViewer","850x800")
app.setBg("light gray")
app.setLocation("CENTER")

location = os.getcwd()+'/bagging/'

for i in range(len(baglist)):
	real_bag = ''
	recordcnt.append(0)
	baglist[i] = baglist[i][1:]
	for j in range(len(baglist[i])):
		if baglist[i][j] == '/':
			real_bag = ''
			continue
		real_bag += baglist[i][j]
	baglist[i] = real_bag
	baglist[i] = '['+ str(i) + '] ' + baglist[i] + '.txt'

try:
	os.mkdir('bagging')
except:
	pass

baglistforauto = baglist[:]
#----------TAP1----------#
def pressTopic(topicecho):
	global subFlag, recordtopic, echo
	recordtopic = topicecho[1:]
	if subFlag is True:
		sub = rospy.get_param('sub')
		sub.unregister()
	rosecho.rostopic_echo(topicecho, rosecho.CallbackEcho(topicecho))
	app.registerEvent(setlabeltopic)
	subFlag = True

def SearchTopic():
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
	echo = rospy.get_param('echo')
	app.setLabel("topic", echo)

def pressrecord(Record):
	global recordFlag
	recordFlag = True

def record():
	global recordcnt, recordFlag, recordtopic
	if recordFlag == True:
		try:
			filet = open(location + baglist[topiclist.index('/'+recordtopic)], 'a')
			filet.write("joonrecord"+str(recordcnt[topiclist.index('/'+recordtopic)])+'\n')
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

app.setPollTime(10)
app.registerEvent(SearchTopic)
app.registerEvent(record)
app.stopTab()
#----------TAP1----------#

#----------TAP2----------#
def SearchBag():
	text = '#' + app.getEntry("e2")
	bagsearch = filelist[:]
	for i in range(len(filelist)):
		bagsearch[i] = '#' + bagsearch[i]
		if len(text) <= len(bagsearch[i]):
			for j in range(len(text)):
				if bagsearch[i][j] == text[j]:
					app.showButton(filelist[i])
				else:
					app.hideButton(filelist[i])
					break
		else:
			app.hideButton(filelist[i])
			
def pressrm(Remove):
	os.system("cd " + location + "&& rm -rf ./*")  
	for i in range(len(filelist)):
			app.hideButton(filelist[i])
	for i in range(len(topiclist)):
		recordcnt[i] = 0

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
app.addAutoEntry("e2", baglistforauto, 0, 0)
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
for i in range(len(baglist)):
	app.setPadding([10,10])
	app.addButton(baglist[i], pressBagging)
	app.setButtonAlign(baglist[i], "left")
	app.setButtonBg(baglist[i], "white")
	app.setButtonFg(baglist[i], "black")
	app.hideButton(baglist[i])
app.stopScrollPane()

app.startPanedFrame("p6")
app.startScrollPane("s4")
app.addLabel("Bag", "Please Click Bagfile")
app.stopScrollPane()
app.stopPanedFrame()
app.stopPanedFrame()

app.stopPanedFrame()
app.registerEvent(Refresh)
app.registerEvent(SearchBag)
app.stopTab()
#----------TAP2----------#
app.stopTabbedFrame()

app.go()
