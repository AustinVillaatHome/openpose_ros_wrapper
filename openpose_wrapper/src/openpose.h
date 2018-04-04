#ifndef __OPENPOSE_H__
#define __OPENPOSE_H__

#include<gflags/gflags.h>
#include <openpose/headers.hpp>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

DECLARE_int32(logging_level);
DECLARE_bool(disable_multi_thread);
DECLARE_int32(profile_speed);
DECLARE_string(image_dir);
DECLARE_string(model_folder);
DECLARE_string(output_resolution);
DECLARE_int32(num_gpu);
DECLARE_int32(num_gpu_start);
DECLARE_int32(keypoint_scale);
DECLARE_bool(body_disable);
DECLARE_string(model_pose);
DECLARE_string(net_resolution);
DECLARE_int32(scale_number);
DECLARE_double(scale_gap);
DECLARE_bool(heatmaps_add_parts);
DECLARE_bool(heatmaps_add_bkg);
DECLARE_bool(heatmaps_add_PAFs);
DECLARE_int32(heatmaps_scale);
DECLARE_bool(part_candidates);
DECLARE_bool(face);
DECLARE_string(face_net_resolution);
DECLARE_bool(hand);
DECLARE_string(hand_net_resolution);
DECLARE_int32(hand_scale_number);
DECLARE_double(hand_scale_range);
DECLARE_bool(hand_tracking);
DECLARE_int32(part_to_show);
DECLARE_bool(disable_blending);
DECLARE_double(render_threshold);
DECLARE_int32(render_pose);
DECLARE_double(alpha_pose);
DECLARE_double(alpha_heatmap);
DECLARE_double(face_render_threshold);
DECLARE_int32(face_render);
DECLARE_double(face_alpha_pose);
DECLARE_double(face_alpha_heatmap);
DECLARE_double(hand_render_threshold);
DECLARE_int32(hand_render);
DECLARE_double(hand_alpha_pose);
DECLARE_double(hand_alpha_heatmap);
DECLARE_string(write_images);
DECLARE_string(write_images_format);
DECLARE_string(write_video);
DECLARE_string(write_json);
DECLARE_string(write_coco_json);
DECLARE_string(write_heatmaps);
DECLARE_string(write_heatmaps_format);
DECLARE_string(write_keypoint);
DECLARE_string(write_keypoint_format);
DECLARE_string(write_keypoint_json);

struct UserDatum : public op::Datum
{
	std_msgs::Header header;

    UserDatum(void)
    {}
};

class OpenposeWrapper
{
private:
    op::Wrapper<std::vector<UserDatum>> opWrapper;
	std::shared_ptr<std::vector<UserDatum> > data;
public:
	OpenposeWrapper(void);
	void start(void) { ROS_INFO("OpWrapper started"); opWrapper.start(); };
	void stop(void) { opWrapper.stop(); };
	std::shared_ptr<std::vector<UserDatum> > process( cv::Mat &);
};
#endif // __OPENPOSE_H__
