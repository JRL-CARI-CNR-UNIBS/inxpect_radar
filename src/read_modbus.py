import rospy
from Inxpect import Inxpect
from inxpect.msg import Radar


def main():
    rospy.init_node("inxpect_reader_node")

    # rospy.get_param("inxpect_devices=_ip")
    hz = 30

    sensor_device = Inxpect(ip="192.168.1.10")

    radar_publisher = rospy.Publisher("radar_data", Radar, queue_size=10)
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        distance, angle = sensor_device.read()

        data_msg = Radar()
        data_msg.distance = distance
        data_msg.angle = angle

        radar_publisher.publish(data_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
