#! /usr/bin/env python3

import rospy
from Inxpect import Inxpect
from inxpect_radar.msg import Radar


def main():
    rospy.init_node("inxpect_reader_node")

    # TODO: Do param
    # rospy.get_param("inxpect_devices=_ip")
    # rospy.get_param("hz")
    # radar_data

    hz = 100

    sensor_device = Inxpect(ip="192.168.1.10", lambda_=0.8)

    radar_publisher = rospy.Publisher("radar_data", Radar, queue_size=10)
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        distance, angle, \
        distance_preproc, angle_preproc, \
        distance_filtered, angle_filtered = sensor_device.read()

        data_msg = Radar()
        data_msg.distance = distance
        data_msg.angle = angle
        data_msg.distance_preproc = distance_preproc
        data_msg.angle_preproc = angle_preproc
        data_msg.distance_filtered = distance_filtered
        data_msg.angle_filtered = angle_filtered

        radar_publisher.publish(data_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
