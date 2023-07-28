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

    ip_adresses = ["192.168.1.10"]
    radars = {}
    radar_publishers = {}

    for i in range(len(ip_adresses)):
        new_device = Inxpect(ip=ip_adresses[i], lambda_=0.8)
        radars.update({"Radar" + str(i+1) : new_device})
        radar_publishers.update({"Radar" + str(i+1) + "_pub" : \
                                 rospy.Publisher("radar" + str(i+1) + "_data", Radar, queue_size=10)})
    
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():

        for i in range(len(ip_adresses)):
            distance, angle, \
            distance_preproc, angle_preproc, \
            distance_filtered, angle_filtered = radars["Radar" + str(i+1)].read_windowed()

            data_msg = Radar()
            data_msg.distance = distance
            data_msg.angle = angle
            data_msg.distance_preproc = distance_preproc
            data_msg.angle_preproc = angle_preproc
            data_msg.distance_filtered = distance_filtered
            data_msg.angle_filtered = angle_filtered

            radar_publishers["Radar" + str(i+1) + "_pub"].publish(data_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
