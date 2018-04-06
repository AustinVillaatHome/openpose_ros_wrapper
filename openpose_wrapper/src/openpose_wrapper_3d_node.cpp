#include <iostream>
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread
#include <unistd.h>
#include <string>
#include "openpose.h"
// OpenPose dependencies
#include <openpose/headers.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pthread.h>
#include <array>
#include <openpose_wrapper/OpenPose.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <openpose_ros_wrapper_msgs/Persons.h>
#include <openpose_ros_wrapper_msgs/Persons3d.h>
#include <openpose_ros_wrapper_msgs/BodyPartDetection.h>
#include <openpose_ros_wrapper_msgs/BodyPartDetection3d.h>
#include <openpose_ros_wrapper_msgs/EyeDetection.h>
#include <openpose_ros_wrapper_msgs/PersonDetection3d.h>
#include <openpose_ros_wrapper_msgs/PersonDetection.h>
#include <image_transport/image_transport.h>
#include <openpose_ros_msgs/GetPersons.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480

//#include <openpose_ros_msgs/.h>

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage out_msg;
openpose_ros_wrapper_msgs::EyeDetection eye_msg;

//added from mk:ToDO
op::CvMatToOpInput *cvMatToOpInput;
op::CvMatToOpOutput *cvMatToOpOutput;
op::PoseExtractorCaffe *poseExtractorCaffe;
op::PoseRenderer *poseRenderer;
op::FaceDetector *faceDetector;
op::FaceExtractor *faceExtractor;
op::FaceRenderer *faceRenderer;
op::OpOutputToCvMat *opOutputToCvMat;

openpose_ros_msgs::BodypartDetection getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.confidence = NAN;
  return bodypart;
}

openpose_ros_msgs::BodypartDetection getBodyPartDetectionFromArrayAndIndex(const op::Array<float>& array, size_t idx)
{
  openpose_ros_msgs::BodypartDetection bodypart;

  bodypart.x = array[idx];
  bodypart.y = array[idx+1];
  bodypart.confidence = array[idx+2];
  return bodypart;
}

//std::map<unsigned int, std::string> getBodyPartMapFromPoseModel(const op::PoseModel& pose_model)
//{
  //if (pose_model == op::PoseModel::COCO_18)
  //{
    //return op::POSE_COCO_BODY_PARTS;
  //}
  //else if (pose_model == op::PoseModel::MPI_15 || pose_model == op::PoseModel::MPI_15_4)
  //{
    //return op::POSE_MPI_BODY_PARTS;
  //}
  //else
  //{
    //ROS_FATAL("Invalid pose model, not map present");
    //exit(1);
  //}
//}


class MyPublisher
{
    public:
    MyPublisher(void);
    ros::Publisher publisher;
    ros::Publisher image_skeleton_pub;
    ros::Publisher pose_pub;
    ros::Publisher pose_3d_pub;
    ros::Publisher keypoints_pub;
    ros::Publisher eye_pub;
    ros::Subscriber pcl_sub;

    openpose_ros_wrapper_msgs::Persons persons;
    openpose_ros_wrapper_msgs::Persons3d persons_3d;
    //point_cloud   
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; 

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void callback(const op::Array<float>&);

    std_msgs::Float32MultiArray msg;
    std::map<unsigned int, std::string> bodypartsmap;
};

MyPublisher mp;

using namespace std;



void workConsumer(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
	ROS_INFO("Process Image\n");
	try
	{
		// User's displaying/saving/other processing here
			// datum.cvOutputData: rendered frame with pose or heatmaps
			// datum.poseKeypoints: Array<float> with the estimated pose
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			// Show in command line the resulting pose keypoints for body, face and hands
			//op::log("\nKeypoints for " + std::to_string(datumsPtr->at(0).index) + " :");
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
			 //
			//op::log(" ");
			// Alternative: just getting std::string equivalent
			//op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());

			const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
			//op::log("Person face keypoints:");

			for (auto face = 0 ; face < faceKeypoints.getSize(0) ; face++)
			{
				//op::log("face " + std::to_string(face) + " (x, y, score):");
				for (auto bodyPart = 0 ; bodyPart < faceKeypoints.getSize(1) ; bodyPart++)
				{

					if((bodyPart >35 ) && (bodyPart <48)){
						std::string valueToPrint;
						for (auto xyscore = 0 ; xyscore < faceKeypoints.getSize(2) ; xyscore++)
						{
							valueToPrint += std::to_string(faceKeypoints[{face, bodyPart, xyscore}]   ) + " ";
						}
						//op::log(valueToPrint);
					}
					else if(bodyPart ==68) //left eye
					{
						float confidence_left = faceKeypoints[{face, bodyPart, 2}];
						//std::string valueToPrint2;
						//valueToPrint2 = "left eye confidence: ";
						//valueToPrint2 += std::to_string(faceKeypoints[{face, bodyPart, 2}]) + " ";
						//op::log(valueToPrint2);
						eye_msg.confidence_left = confidence_left;
						if(confidence_left>0.7)
							eye_msg.left_eye=true;
						else
							eye_msg.left_eye=false;
					}
					else if(bodyPart ==69) //right eye
					{
						//std::string valueToPrint2;
						//valueToPrint2 = "right eye confidence: ";
						//valueToPrint2 += std::to_string(faceKeypoints[{face, bodyPart, 2}]) + " ";
						//op::log(valueToPrint2);
						float confidence_right= faceKeypoints[{face, bodyPart, 2}];
						eye_msg.confidence_right= confidence_right;
						if(confidence_right>0.7)
							eye_msg.right_eye=true;
						else
							eye_msg.right_eye=false;

					}
					else{
					
					}
				}
			}
			//op::log(" ");

			mp.callback(poseKeypoints);

			// Display rendered output image
			//cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
			out_msg.image=datumsPtr->at(0).cvOutputData;
		}
	}
	catch (const std::exception& e)
	{
		op::log("Some kind of unexpected error happened.");
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

void sync_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg)
{

    ROS_INFO("syncronized callback");
    ROS_INFO("cloud time stamp :L %f ", cloud_msg->header.stamp.toSec());
    ROS_INFO("image time stamp :L %f ", image_msg->header.stamp.toSec());

    //ROS_INFO("syncronized callback");
    //image subscriber
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //save to pointcloud
    pcl::fromROSMsg(*cloud_msg, *(mp.cloud));
}



void callback(const sensor_msgs::Image &img)
{
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);


    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    /*ToDo 
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    op::Array<float> outputArray;

    // process
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput->format(cv_ptr->image);
    double scaleInputToOutput;
    std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput->format(cv_ptr->image);
    // Step 3 - Estimate poseKeypoints
    poseExtractorCaffe->forwardPass(netInputArray, {cv_ptr->image.cols, cv_ptr->image.rows}, scaleRatios);
    const auto poseKeypoints1 = poseExtractorCaffe->getPoseKeypoints();
    const auto faces = faceDetector->detectFaces(poseKeypoints1, scaleInputToOutput);
    faceExtractor->forwardPass(faces, cv_ptr->image, scaleInputToOutput);
    const auto faceKeypoints = faceExtractor->getFaceKeypoints();
    */



//  cout << "New image " << img.width << "x" << img.height << " " << img.data.size() << endl;
}

//MyPublisher::MyPublisher(void)
MyPublisher::MyPublisher(void):cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    
    //bodypartsmap = op::POSE_COCO_BODY_PARTS;
    //bodypartsmap = op::getPoseBodyPartMapping(pose_model);
}


void MyPublisher::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){

    //ROS_INFO("cloud callback");
    pcl::fromROSMsg(*msg, *cloud);

}

void MyPublisher::callback(const op::Array<float> &poseKeypoints)
{
    ros::Time t = ros::Time::now();
//  openpose_wrapper::OpenPose msg;
    //openpose_ros_wrapper_msgs::Persons persons;
    
    persons=openpose_ros_wrapper_msgs::Persons();
    persons.rostime = t;
    persons.image_w = 640;
    persons.image_h = 480;

    persons_3d=openpose_ros_wrapper_msgs::Persons3d();
    persons_3d.rostime = t;
    persons_3d.image_w = 640;
    persons_3d.image_h = 480;

   //if(cloud_temp->points.size() == 0)
       //return;
//copied 

// original msgs_mk
//#####################################################################################3
    const int num_people = poseKeypoints.getSize(0);
    const int num_bodyparts = poseKeypoints.getSize(1);

    std::vector<int> num_count(3,0);
    std::vector<float> mean_dist(3,0.0);

    for(size_t person_idx = 0; person_idx < num_people; person_idx++) {
        openpose_ros_wrapper_msgs::PersonDetection person;
        openpose_ros_wrapper_msgs::PersonDetection3d person_3d;


        for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
        {
        
            num_count[c_idx]=0;
            mean_dist[c_idx]=0.0;
        
        }

        for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {

            size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
            openpose_ros_wrapper_msgs::BodyPartDetection bodypart;
            bodypart.part_id = bodypart_idx;
            bodypart.x = poseKeypoints[final_idx];
            bodypart.y = poseKeypoints[final_idx+1];
            bodypart.confidence = poseKeypoints[final_idx+2];
            person.body_part.push_back(bodypart);

            openpose_ros_wrapper_msgs::BodyPartDetection3d bodypart_3d;
            bodypart_3d.part_id = bodypart_idx;
            bodypart_3d.x = poseKeypoints[final_idx];
            bodypart_3d.y = poseKeypoints[final_idx+1];
            //bodypart_3d.z = 0 ;
            bodypart_3d.confidence = poseKeypoints[final_idx+2];

            //extract z information
            if(bodypart_3d.x>0 && bodypart_3d.y>0){

                int point_idx = bodypart.x+bodypart.y*CAMERA_PIXEL_WIDTH;
                float point_z=0.0;
                float point_x=0.0;
                float point_y=0.0;

                if ((point_idx<0) || (!pcl::isFinite(cloud->points[point_idx]))){
                    continue;
                }
                else{

                   if(cloud->points[point_idx].z) 
                   {
                       point_x = cloud->points[point_idx].x;
                       point_y = cloud->points[point_idx].y;
                       point_z = cloud->points[point_idx].z;

                   }
                    //ROS_INFO("bodyparpoint x : %d , y: %d , point idx :%d , point_z : %.3f ",bodypart_3d.x, bodypart_3d.y, point_idx, point_z);
                    bodypart_3d.x =point_x; 
                    bodypart_3d.y =point_y; 
                    bodypart_3d.z =point_z; 

                    for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
                        num_count[c_idx]++;
                    
                    mean_dist[0]+=bodypart_3d.x;
                    mean_dist[1]+=bodypart_3d.y;
                    mean_dist[2]+=point_z;
                    
                }
                //ROS_INFO("bodyparpoint x : %d , y: %d , point idx :%d , point_z : %d ",bodypart_3d.x, bodypart_3d.y, point_idx, point_3d.z);
            }
            else{
                bodypart_3d.z=0;
            }
            
            person_3d.body_part.push_back(bodypart_3d);

        }
        
        //calculate average_distance of body parts
        for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
        {
        
            if(num_count[c_idx]!=0)
                mean_dist[c_idx] =static_cast<float>(mean_dist[c_idx]/num_count[c_idx]);
            else
                mean_dist[c_idx]=0.0;

        }

        //mean_z =s;tatic_cast<float>(mean_z/z_count);
        //person_3d.avg_pose.header.frame_id = (*cloud).header.frame_id;
        person_3d.avg_pose.header.frame_id = "head_rgbd_sensor_rgb_frame";
        person_3d.avg_pose.pose.position.x= mean_dist[0];
        person_3d.avg_pose.pose.position.y= mean_dist[1];
        person_3d.avg_pose.pose.position.z= mean_dist[2];
        person_3d.avg_pose.pose.orientation.x= 0.0;
        person_3d.avg_pose.pose.orientation.y= 0.0;
        person_3d.avg_pose.pose.orientation.z= 0.0;
        person_3d.avg_pose.pose.orientation.w= 1.0;
        
        persons.persons.push_back(person);
        persons_3d.persons.push_back(person_3d);
    }

    pose_pub.publish(persons);
    pose_3d_pub.publish(persons_3d);
 

    //int numHuman = poseKeypoints.getSize(0);
    //int numPart  = poseKeypoints.getSize(1);
    //int numInfo  = poseKeypoints.getSize(2);

    //mp.msg.layout.dim.clear();
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim[0].size = numHuman;
    //mp.msg.layout.dim[0].stride = numPart*numInfo;
    //mp.msg.layout.dim[0].label = "human";
    //mp.msg.layout.dim[1].size = numPart;
    //mp.msg.layout.dim[1].stride = numInfo;
    //mp.msg.layout.dim[1].label = "body";
    //mp.msg.layout.dim[2].size = numInfo;
    //mp.msg.layout.dim[2].stride = 1;
    //mp.msg.layout.dim[2].label = "info";
    //mp.msg.data.resize(numHuman*numPart*numInfo);

    //op::log("DIM " + std::to_string(numHuman) + std::to_string(numPart) + std::to_string(numInfo));
    //for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
    //{
        //op::log("Person " + std::to_string(person) + ": (x, y, score):");
        //for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
        //{
            //float x, y, p;

            //x = poseKeypoints[{person, bodyPart, 0}];
            //y = poseKeypoints[{person, bodyPart, 1}];
            //p = poseKeypoints[{person, bodyPart, 2}];

//#if 0
//#else
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 0 ] = x;
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 1 ] = y;
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 2 ] = p;
//#endif
        //}
    //}
    //publisher.publish(mp.msg);

//#####################################################################################3

    //publish image
    //sensor_msgs::Image ros_image;
    //out_msg.encoding =sensor_msgs::image_encodings::BGR8;
    //image_skeleton_pub.publish(out_msg.toImageMsg());

    //eye_pub.publish(eye_msg);

}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "openpose_wrapper");
    ros::NodeHandle nh;

    ros::start();
//  mp.publisher = nh.advertise<openpose_wrapper::OpenPose>("openpose_human_body", 1000);
    mp.publisher = nh.advertise<std_msgs::Float32MultiArray>("openpose_human_body", 1000);
    mp.image_skeleton_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/detected_poses_image", 1 );  
    mp.pose_pub = nh.advertise<openpose_ros_wrapper_msgs::Persons>("/openpose/pose", 2);
    mp.pose_3d_pub = nh.advertise<openpose_ros_wrapper_msgs::Persons3d>("/openpose/pose_3d", 2);
    mp.keypoints_pub = nh.advertise<openpose_ros_msgs::PersonDetection>( "/openpose_ros/skeleton_3d/detected_poses_keypoints" , 0 );
    mp.eye_pub = nh.advertise<openpose_ros_wrapper_msgs::EyeDetection>( "/openpose_ros/eye_detections" , 0 );

    //mp.pcl_sub= nh.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10,
                                       //&MyPublisher::cloud_callback, &mp);
    //ros::Subscriber subscriber = nh.subscribe( FLAGS_image_dir, 1, callback);
    //ros::Subscriber subscriber = nh.subscribe( "/hsrb/head_rgbd_sensor/rgb/image_raw", 1, callback);
    //ros::Subscriber subscriber = nh.subscribe( FLAGS_image_dir, 1, callback);
    
    //message_filters::Subscriber<sensor_msgs::Image> hsr_image_sub(nh,"/hsrb/head_rgbd_sensor/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> hsr_image_sub(nh,FLAGS_image_dir, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> hsr_pcl_sub(nh,"/hsrb/head_rgbd_sensor/depth_registered/rectified_points",1);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),hsr_pcl_sub,hsr_image_sub);
    sync.registerCallback(boost::bind(&sync_callback,_1,_2));

	OpenposeWrapper wrapper;

	wrapper.start();

	while ( ros::ok() )
	{
		ros::spinOnce();

		if ( cv_ptr == nullptr )
			continue;

		auto ret = wrapper.process(cv_ptr->image);
		if ( ret )
		{
			workConsumer(ret);
		}
		cv_ptr = nullptr;
	}

	wrapper.stop();

    ros::shutdown();

    return 0;
}
