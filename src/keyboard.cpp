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


std::string exeCMD(const char* cmd){
	char buffer[128];
	std::string result="";
	FILE* pipe=popen(cmd,"r");
	if(!pipe) throw std::runtime_error("popen() failed");
	try{
		while(fgets(buffer,sizeof(buffer),pipe)!=NULL){
			result+=buffer;
		}
	}
	catch(...){
		pclose(pipe);
		throw;
	}
	pclose(pipe);
	return result;
}


std::string keyboard_eventX = "";
std::string keyboard_path = "";


static void initKeyboardPermission(){

	std::string keyboard_cmd="ls -l /dev/input/by-path/*event-kbd";

	cout<<"Executing: "+keyboard_cmd<<endl;

	std::string cmdRet=exeCMD(keyboard_cmd.c_str());

	cout<<"Keyboard Permission Status:"<<endl<<cmdRet<<endl;


	std::stringstream cmdRetStream(cmdRet);
	vector<string> tokens;
	string intermediate;
	while(getline(cmdRetStream,intermediate,' ')){
		tokens.push_back(intermediate);
	}
	keyboard_path=tokens[tokens.size()-3];
	keyboard_eventX=tokens[tokens.size()-1].substr(tokens[tokens.size()-1].find("event"));
	cout<<"Keyboard ID: "<<keyboard_path<<"  "<<keyboard_eventX<<endl;
	
	string cmdPermissionString="sudo chmod a+rw /dev/input/"+keyboard_eventX;
	cout<<"Excecuting: "+cmdPermissionString<<endl;
	system(cmdPermissionString.c_str());

	
	cmdRet=exeCMD(keyboard_cmd.c_str());

	cout<<"Keyboard Permission Status Changed to:"<<endl<<cmdRet<<endl;

	cout<<"Waiting for input..."<<endl;

}

int main(int argc, char** argv)
{
	int fd;
	struct input_event event;
    const char *path;
	
	printf("This is a keyboard input device demo. \r\n");

	initKeyboardPermission();
	
  	path = keyboard_path.c_str();

 	fd = open(path, O_RDONLY);

	if (fd < 0)
	{
		printf("Failed to open device %s. \n"
			"Please confirm the path or you have permission to do this. \n", path);
		exit(1);
	}
	
	ros::init(argc, argv, "keyboard");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<origarm_ros::keynumber>("key_number",100);
	ros::Rate r(100);
	origarm_ros::keynumber keycode;
	while (ros::ok())
	{		
		if (read(fd, &event, sizeof(event)) == sizeof(event))
		{
			//if (event.type != EV_SYN && event.code != MSC_SCAN)
			if (event.type == EV_KEY)
			{
				//two lines output including pressing value == 1, & releasing value == 0, long time pressing value == 2
				printf("Event: time %ld.%ld, type %d, code %d, value %d\n",
					event.time.tv_sec,event.time.tv_usec,
					event.type,
					event.code,
					event.value);

				if (event.value==0){
					keycode.keycodePressed=	event.code;
					pub.publish(keycode);
					printf("KEY %d pressed\n", keycode.keycodePressed);
				}
			}
		}

		ros::spinOnce();
		r.sleep();
	}
	
	close (fd);

	return 0;

}
