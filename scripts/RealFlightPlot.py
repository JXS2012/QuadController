#!/usr/bin/env python
import rospy
import roslib
import tf

import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math

def main():
    rospy.init_node('flight_plot')

    listener = tf.TransformListener()

    rate = rospy.Rate(30.0)

    x = []
    y = []
    z = []

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world','/vicon/bird/bird',rospy.Time(0))
            x.append(trans[0])
            y.append(trans[1])
            z.append(trans[2])
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            continue
    
        fig1 = plt.figure(1)
        ax = Axes3D(fig1)
        legend = []
        for i in range(flight_no-1,flight_no):
            ax.plot(x[i],y[i],z[i])
            legend.append('{0}'.format(i))
        plt.legend(legend)

        fig2 = plt.figure(2)
        legend = []
        for i in range(flight_no-1,flight_no):
            plt.plot(range(len(x[i])),x[i],'ro')
            legend.append('{0}'.format(i))
        plt.legend(legend)

        fig3 = plt.figure(3)
        legend = []
        for i in range(flight_no-1,flight_no):
            plt.plot(range(len(y[i])),y[i],'ro')
            legend.append('{0}'.format(i))
        plt.legend(legend)

        fig4 = plt.figure(4)
        legend = []
        for i in range(flight_no-1,flight_no):
            plt.plot(range(len(z[i])),z[i],'ro')
            legend.append('{0}'.format(i))
        plt.legend(legend)

        plt.show()

        rate.sleep();

if __name__=='__main__':
    main()
