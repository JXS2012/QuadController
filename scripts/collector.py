#!/usr/bin/env python
import roslib; roslib.load_manifest('quadrotorTestControl')
import rospy
from std_msgs.msg import Float32
import serial

if __name__ == '__main__':
    try:
        rospy.init_node('collector')
    except rospy.ROSInterruptException:
        pass
	
	#black cat
	#ser = serial.Serial('COM9',57600)
	#ser = serial.Serial('/dev/ttyUSB0',57600)
	#sparkfun
	ser = serial.Serial('/dev/ttyUSB0',9600)
	if (not ser.isOpen()):
    	ser.open()
  
	initial_time = time.time()
	print initial_time
	current_time = initial_time
	timetable=[]

	while not rospy.is_shutdown():
		ser.read()
	    current_time = rospy.get_time()
	    timetable.append(current_time)
	    rospy.loginfo('gamma ray received at {0}'.format(current_time))
  
	timetable.pop()
	count = [0]*(int(timetable[-1]/60)+1)
	for digit in timetable:
    	count[int(digit/60)]+=1
  
	print count
	print timetable
  
	filename = '{0}cm_{1}min.txt'.format(r,T)
	f=open(filename,'w')
	for digit in timetable:
    	f.write(str(digit)+'\n')
	f.close()
  
	filename = '{0}cm_{1}min_count.txt'.format(r,T)
	f=open(filename,'w')
	for digit in count:
    	f.write(str(digit)+'\n')
	f.close()
  
	ser.close() 

    print("Closing...")
