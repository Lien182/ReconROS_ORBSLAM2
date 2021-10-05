
#include <iostream>
#include <sstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

using namespace std;

#define debug(...)

extern "C" {
	#include "../lib/comp/ros.h"
	#include "../lib/comp/ros_sub.h"
	#include "../lib/comp/ros_pub.h"
}




#define ITERATIONS	10

#define ROS_TYPE_MESSAGE_TYPE rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image()

#define OFFSETOF(type, member) ((uint32_t)(intptr_t)&(((type *)(void*)0)->member) )


struct ros_node_t 		resources_rosnode;
struct ros_publisher_t 	resources_pubdata[2];


struct timespec t1, t2;


void* node_thread(void * arg)
{
	int i = 0;
	int img_cnt = 0;

	sensor_msgs__msg__Image * image_left = sensor_msgs__msg__Image__create();
	sensor_msgs__msg__Image * image_rigt = sensor_msgs__msg__Image__create();



	if(ros_node_init(&resources_rosnode, "rosorb_client") < 0)
	{
		printf("Thread 0: ROS Node init failed \n");
		return (void*)-1;
	}


	if( ros_publisher_init(&resources_pubdata[0], &resources_rosnode, ROS_TYPE_MESSAGE_TYPE, "/stereo/left/image_raw") < 0 )
	{
		ros_node_destroy(&resources_rosnode);
		return (void*)-1;
	}
	if( ros_publisher_init(&resources_pubdata[0], &resources_rosnode, ROS_TYPE_MESSAGE_TYPE, "/stereo/right/image_raw") < 0 )
	{
		ros_node_destroy(&resources_rosnode);
		return (void*)-1;
	}
	
	//usleep(200000);	

	cv::Mat imLeft, imRight;
	for(i = 0; i < ITERATIONS; i++ )
	{
		

		stringstream ss;
		ss << setfill('0') << setw(6) << img_cnt;
		string strImageLeft  = string("00/image_0/") + ss.str() + ".png";
		string strImageRight = string("00/image_1/") + ss.str() + ".png";

		img_cnt++;


		// Read left and right images from file
		imLeft = cv::imread(strImageLeft,CV_LOAD_IMAGE_UNCHANGED);
		imRight = cv::imread(strImageRight,CV_LOAD_IMAGE_UNCHANGED);
		//double tframe = vTimestamps[ni];

		if(imLeft.empty())
		{
			cerr << endl << "Failed to load image at: " << string(strImageLeft) << endl;
			return 1;
		}

	}

	ros_publisher_destroy(&resources_pubdata[0]);
	ros_publisher_destroy(&resources_pubdata[1]);

	ros_node_destroy(&resources_rosnode);

	return (void*)0;
}


int main(int argc, char **argv) 
{
	pthread_t p1;

	pthread_create(&p1, NULL, &node_thread, NULL);

  	pthread_join (p1, NULL);

	return 0;
}