// ------------------------- OpenPose Library Tutorial - Wrapper - Example 2 - Synchronous -------------------------
// Synchronous mode: ideal for performance. The user can add his own frames producer / post-processor / consumer to the OpenPose wrapper or use the
// default ones.

// This example shows the user how to use the OpenPose wrapper class:
    // 1. User reads images
    // 2. Extract and render keypoint / heatmap / PAF of that image
    // 3. Save the results on disk
    // 4. User displays the rendered pose
    // Everything in a multi-thread scenario
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module:
        // For the Array<float> class that the `pose` module needs
        // For the Datum struct that the `thread` module sends between the queues
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <iostream>
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread
#include <unistd.h>
#include <string>
// Other 3rdparty dependencies
// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include "openpose.h"
// Allow Google Flags in Ubuntu 14
// OpenPose dependencies
#include <openpose/headers.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <pthread.h>
#include <array>
//#include <openpose_wrapper/OpenPose.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <openpose_ros_wrapper_msgs/Persons.h>
#include <openpose_ros_wrapper_msgs/BodyPartDetection.h>
#include <openpose_ros_wrapper_msgs/PersonDetection.h>
#include <image_transport/image_transport.h>
#include <openpose_ros_wrapper_msgs/GetPersons.h>

cv_bridge::CvImage out_msg;

//added from mk:ToDO
op::CvMatToOpInput *cvMatToOpInput;
op::CvMatToOpOutput *cvMatToOpOutput;
op::PoseExtractorCaffe *poseExtractorCaffe;
op::PoseRenderer *poseRenderer;
op::FaceDetector *faceDetector;
op::FaceExtractor *faceExtractor;
op::FaceRenderer *faceRenderer;
op::OpOutputToCvMat *opOutputToCvMat;

class MyPublisher
{
	public:
	MyPublisher(void);
	OpenposeWrapper wrapper;
	ros::Publisher image_skeleton_pub;
	ros::ServiceServer server;
    openpose_ros_wrapper_msgs::Persons callback(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr);
};

pthread_mutex_t buf_mutex = PTHREAD_MUTEX_INITIALIZER;
bool buff_empty = true;
MyPublisher *mp;

using namespace std;

void workConsumer(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
}

MyPublisher::MyPublisher(void)
{
}

openpose_ros_wrapper_msgs::Persons MyPublisher::callback(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
    ros::Time t = ros::Time::now();
    openpose_ros_wrapper_msgs::Persons persons;
    persons.rostime = t;
    persons.image_w = 640;
    persons.image_h = 480;

	try
	{
		// User's displaying/saving/other processing here
			// datum.cvOutputData: rendered frame with pose or heatmaps
			// datum.poseKeypoints: Array<float> with the estimated pose
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			// Show in command line the resulting pose keypoints for body, face and hands
//			op::log("\nKeypoints for " + std::to_string(datumsPtr->at(0).index) + " :");
			// Accesing each element of the keypoints
			const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
			//op::log("Person pose keypoints:");
			//for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
			//{
				//op::log("Person " + std::to_string(person) + " (x, y, score):");
				//for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
				//{
					//std::string valueToPrint;
					//for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
					//{
						//valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
					//}
					//op::log(valueToPrint);
				//}
			//}
			//op::log(" ");
             //Alternative: just getting std::string equivalent
			//op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());
			//op::log("Left hand keypoints: " + datumsPtr->at(0).handKeypoints[0].toString());
			//op::log("Right hand keypoints: " + datumsPtr->at(0).handKeypoints[1].toString());
			// Heatmaps
			//const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
			//if (!poseHeatMaps.empty())
			//{
				//op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
						//+ std::to_string(poseHeatMaps.getSize(1)) + ", "
						//+ std::to_string(poseHeatMaps.getSize(2)) + "]");
				//const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
				//op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
						//+ std::to_string(faceHeatMaps.getSize(1)) + ", "
						//+ std::to_string(faceHeatMaps.getSize(2)) + ", "
						//+ std::to_string(faceHeatMaps.getSize(3)) + "]");
				//const auto& handHeatMaps = datumsPtr->at(0).handHeatMaps;
				//op::log("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
						//+ std::to_string(handHeatMaps[0].getSize(1)) + ", "
						//+ std::to_string(handHeatMaps[0].getSize(2)) + ", "
						//+ std::to_string(handHeatMaps[0].getSize(3)) + "]");
				//op::log("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
						//+ std::to_string(handHeatMaps[1].getSize(1)) + ", "
						//+ std::to_string(handHeatMaps[1].getSize(2)) + ", "
						//+ std::to_string(handHeatMaps[1].getSize(3)) + "]");
			//}

			const int num_people = poseKeypoints.getSize(0);
			const int num_bodyparts = poseKeypoints.getSize(1);

			for(size_t person_idx = 0; person_idx < num_people; person_idx++) {
				openpose_ros_wrapper_msgs::PersonDetection person;
				for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {
					size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
					openpose_ros_wrapper_msgs::BodyPartDetection bodypart;
					bodypart.part_id = bodypart_idx;
					bodypart.x = poseKeypoints[final_idx];
					bodypart.y = poseKeypoints[final_idx+1];
					bodypart.confidence = poseKeypoints[final_idx+2];
					person.body_part.push_back(bodypart);
				}
				persons.persons.push_back(person);
			}

			if ( FLAGS_publish_images )
			{
				//publish image
                //out_msg.image=datumsPtr->at(0).cvOutputData;
                //out_msg.encoding =sensor_msgs::image_encodings::BGR8;
                //image_skeleton_pub.publish(out_msg.toImageMsg());
			}
		}
	}
	catch (const std::exception& e)
	{
		op::log("Some kind of unexpected error happened.");
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}

	return persons;
}

bool add(openpose_ros_wrapper_msgs::GetPersons::Request &req, openpose_ros_wrapper_msgs::GetPersons::Response &res)
{
	bool ret;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	auto output = mp->wrapper.process(cv_ptr->image);
	if ( output )
	{
		res.persons = mp->callback(output);
		ret = true;
	}
	else
	{
		ret = false;
	}

	return true;
}

// Default modes can be modified here
//	FLAGS_body_disable = true;
//	FLAGS_face = true;
//	FLAGS_hand = true;
//	FLAGS_publish_images = false;

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    //FLAGS_publish_images = false;

	ros::init(argc, argv, "openpose_server");
	ros::NodeHandle nh;

	ros::start();

	mp = new MyPublisher;

	mp->wrapper.start();

	mp->server = nh.advertiseService("/openpose/GetPersons", add);

	if ( FLAGS_publish_images )
	    mp->image_skeleton_pub = nh.advertise<sensor_msgs::Image>( "/openpose/detected_poses_image", 1 );  

	ros::spin();

	mp->wrapper.stop();
	ros::shutdown();

	return 0;
}
