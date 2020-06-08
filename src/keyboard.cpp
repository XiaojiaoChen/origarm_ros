#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include "origarm_ros/keynumber.h"

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>

using namespace std;

//const char defualt_path[] = "/dev/input/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-event-kbd"; //keyboard
//const char defualt_path[] = "/dev/input/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-event-kbd"; //keyboard
const char defualt_path[] = "/dev/input/by-path/platform-i8042-serio-0-event-kbd"; //AT Translated Set 2 keyboard

int keycode;
int value[10];

int main(int argc, char** argv)
{
	int fd;
	struct input_event event;
	char *path;

	printf("This is a keyboard input device demo. \r\n");

	/*if (argc > 1)
		path = argv[1];
	else
		path = (char *)defualt_path;*/

	path = (char *)defualt_path;

	fd = open(path, O_RDONLY);
	printf("fd: %d\r\n", fd);

	if (fd < 0)
	{
		printf("Failed to open device %s. \n"
			"Please confirm the path or you have permission to do this. \n", path);
		exit(1);
	}
	printf("Test device: %s. \nWaiting for input...\n", path);

	ros::init(argc, argv, "keyboard");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<origarm_ros::keynumber>("key_number",100);
	ros::Rate r(100);
	

	while (ros::ok())
	{		
		if (read(fd, &event, sizeof(event)) == sizeof(event))
		{
			//if (event.type != EV_SYN && event.code != MSC_SCAN)
			if (event.type == EV_KEY)
			{
				//two lines output including pressing value == 1, & releasing value == 0, long time pressing value == 2
				/*printf("Event: time %ld.%ld, type %d, code %d, value %d\n",
					event.time.tv_sec,event.time.tv_usec,
					event.type,
					event.code,
					event.value);*/

				/*printf("Event: type %d, code %d, value %d\n",
					event.type,
					event.code,
					event.value);*/
				
				if (event.code == KEY_0)
				{
					keycode = 0;
				}				
				else
				{
					keycode = (int16_t)event.code - 1; 
				}

				value[keycode] = event.value;
									
			}
		}

		//std_msgs::Int16 keyboard;		
		origarm_ros::keynumber key_number;
		for (int i = 0; i < 10; i++)
		{
			key_number.KEY_CODE[i] = value[i];
		}					

		pub.publish(key_number);

		ros::spinOnce();
		r.sleep();
	}
	
	close (fd);

	return 0;

}
