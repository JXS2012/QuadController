#!/usr/bin/env python
import roslib; roslib.load_manifest('quadrotorTestControl')
import rospy
from std_msgs.msg import Float32
from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.InterfaceKit import *

height = 0

def talker():
    pub = rospy.Publisher('sonar', Float32)
    rospy.init_node('sonar')
    while not rospy.is_shutdown():
        rospy.loginfo(height)
        pub.publish(Float32(height))
        rospy.sleep(1.0)


def AttachHandler(event):
    attachedDevice = event.device
    serialNumber = attachedDevice.getSerialNum()
    deviceName = attachedDevice.getDeviceName()
    attachedDevice.setRatiometric(True)
    attachedDevice.setSensorChangeTrigger(5,1)
    print("Hello to Device " + str(deviceName) + ", Serial Number: " + str(serialNumber))

def DetachHandler(event):
    detachedDevice = event.device
    serialNumber = detachedDevice.getSerialNum()
    deviceName = detachedDevice.getDeviceName()
    print("Goodbye Device " + str(deviceName) + ", Serial Number: " + str(serialNumber))

def sensorChanged(e):
#    f = open('sonar.txt','a')
    height = e.value*1024.0/1000
#    print ("Sensor %i: %i" % (e.index, height))
#    f.write('Height at %.3f\n',height)
#    f.close()
    print ("Sensor %i: %i" % (e.index, height))
    if not rospy.is_shutdown():
        rospy.loginfo(height)
        pub.publish(Float32(height))
    return 0
 
def LocalErrorCatcher(event):
    print("Phidget Exception: " + str(e.code) + " - " + str(e.details) + ", Exiting...")
    exit(1)


if __name__ == '__main__':

#    f = open('sonar.txt','a')
#    f.write('********************************\n')
#    f.close()

    # Create
    try:
        device = InterfaceKit()	
    except RuntimeError as e:
        print("Runtime Error: %s" % e.message)
        
    try:
        device.setOnAttachHandler(AttachHandler)
        device.setOnDetachHandler(DetachHandler)
        device.openPhidget()
        device.setOnSensorChangeHandler(sensorChanged)
    except PhidgetException as e: 
        LocalErrorCatcher(e)

    try:
        pub = rospy.Publisher('sonar', Float32)
        rospy.init_node('sonar')
    except rospy.ROSInterruptException:
        pass

    print("Phidget Simple Playground (plug and unplug devices)");
    print("Press Enter to end anytime...");
    character = str(raw_input())
    
    print("Closing...")
 #   f = open('sonar.txt','a')
 #   f.write('********************************\n')
 #   f.close()
    try:
        device.closePhidget()
    except PhidgetException as e:
        LocalErrorCatcher(e)
