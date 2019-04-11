ROS driver for FITXXX laser.

User Guide:
	1. move FIT-XXX file into ros workspace
	2. run catkin_make in terminal in ros workspace
	3. modify host_ip and port num for your laser.
	4. set your Ethernet IPv4 address to 192.168.1.xxx (different than your host_ip and Netmask to 255.255.255.0
	5. roslaunch fitxxx FITXXX.launch or roslaunch fitxxx FITXXX_display.launch to start fitxxx ros node.


ROS topic: sensor_msgs/LaserScan -> "/scan"

TF frame: /laser (default)

ROS Service: 
	1. "/fitxxx/disconnect_laser_srv"
		* disconnect laser, require restart the node to reconnect laser.

	2. "/fitxxx/start_laser_srv"
		* start receiving continous sensor_msgs/LaserScan.

	3. "/fitxxx/stop_laser_srv"
		* stop receiving continous sensor_msgs/LaserScan.
		* example: rosservice call /fitxxx/stop_laser_srv