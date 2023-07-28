#! /usr/bin/env python3

import rospy
from Inxpect import Inxpect
from InxpectNetwork import InxpectNetwork
from inxpect_radar.msg import Radar


def main():
    rospy.init_node("inxpect_reader_node")

    # Read params
    ip_address = rospy.get_param("~ip")     # address of the control unit
    hz = rospy.get_param("~hz")             # node frequency
    lambda_ = rospy.get_param("~lambda")    # observer gain constant

    # Create object to store the network of radars
    radar_network = InxpectNetwork(ip=ip_address)

    # MODBUS fiels to be read
    modbus_fields = {"Radar1" : {"distance": 41014, "angle": 41015},
                     "Radar2" : {"distance": 41022, "angle": 41023}}
    
    # modbus_fields = {"Radar1" : {"distance": 41014, "angle": 41015},
    #                  "Radar2" : {"distance": 41022, "angle": 41023},
    #                  "Radar3" : {"distance": 41030, "angle": 41031},
    #                  "Radar4" : {"distance": 41038, "angle": 41039}}
    
    # ROS publishers to log sensor data
    radar_publishers = {}

    for i in range(len(modbus_fields)):
        new_device = Inxpect(name="Radar" + str(i+1), \
                             client=radar_network.client, \
                             modbus_fields=modbus_fields["Radar" + str(i+1)], \
                             lambda_=lambda_)
        radar_network.add_radar(new_device)
        radar_publishers.update({"Radar" + str(i+1) + "_pub" : \
                                 rospy.Publisher("radar" + str(i+1) + "_data", Radar, queue_size=10)})

    
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():

        # Read and publish data from each sensor
        for i in range(radar_network.n_radars):
            radar_network.update_state()

            data_msg = Radar()
            data_msg.distance = radar_network.state["Radar" + str(i+1)]["distance"]
            data_msg.distance = radar_network.state["Radar" + str(i+1)]["angle"]
            data_msg.distance_preproc = radar_network.state["Radar" + str(i+1)]["distance_preproc"]
            data_msg.distance_preproc = radar_network.state["Radar" + str(i+1)]["distance_preproc"]
            data_msg.distance_filtered = radar_network.state["Radar" + str(i+1)]["distance_filtered"]
            data_msg.distance_filtered = radar_network.state["Radar" + str(i+1)]["distance_filtered"]

            radar_publishers["Radar" + str(i+1) + "_pub"].publish(data_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
