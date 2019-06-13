#ifndef DETECTOR_MANAGER_H_INCLUDED
#define DETECTOR_MANAGER_H_INCLUDED

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <mr2_ros/CourtMode.h>
#include <mr2_ros/Point.h>

#include <cmath>
#include <boost/foreach.hpp>
#include <unordered_map>

#include "type.h"
#include "Hough.h"
#include "detector.h"

class line_detector_manager{
    private:
        ros::NodeHandle nh_;
        ros::Publisher field_marker_pub_;
        ros::Subscriber bno_angle_sub_;

        ros::ServiceServer court_mode_srv_server_;
        ros::ServiceServer initial_point_srv_server_;

        first_wall_detector *first_wall_;
        third_parallel_detector *third_parallel_;
        step_forward_detector *step_forward_;
        step_over_detector *step_over_;
        step_detector *step_;
        end_parallel_detector *end_parallel_;
        mt_inner_detector *mt_inner_;

        double bno_angle_;

        line_map field_line_;
        line_map course_line_;

        tf::TransformBroadcaster br_;

        point point_;

        court_mode_t court_mode_;

    public:
        line_detector_manager();
        void update(const Hough &hough);

        void bno_angle_cb(std_msgs::Float32 angle){
            bno_angle_ = angle.data;
        }

        ///コートモードの変更を受け取る
        bool set_court_mode(mr2_ros::CourtMode::Request &req, mr2_ros::CourtMode::Response &res)
        {
            std::string mode;
            switch (req.court_mode.data)
            {
                case COURT_MODE_RED:
                    mode = "RED";
                    break;

                case COURT_MODE_BLUE:
                    mode = "BLUE";
                    break;

                case COURT_MODE_RED_MT:
                    mode = "RED_MT";
                    break;

                case COURT_MODE_BLUE_MT:
                    mode = "BLUE_MT";
                    break;
            }
            ROS_INFO_STREAM("Call court mode(line detector) " << mode);

            court_mode_ = (court_mode_t)req.court_mode.data;
            field_line_create();
            res.success = true;
            return true;
        }

        bool set_initial_point(mr2_ros::Point::Request &req, mr2_ros::Point::Response &res)
        {
            ROS_INFO_STREAM("Call initial point " << req.point.x << " , " << req.point.y << " , " << req.point.z);
            point_ = point(req.point.x, req.point.y, req.point.z);
            res.success = true;
            point_broadcaster();
            double tmp = 0;
            lpf("internal_initialize", tmp, 0); ///LPFの内部配列の初期化
            return true;
        }

        void point_broadcaster(){
            br_.sendTransform(tf::StampedTransform(point_.get_tf_transform(), ros::Time::now(), "map", "base_footprint"));
        }

    private:
        void field_line_create(bool line_update_flag = true);

        double lpf(std::string name, double &value, double a = 0.9);
        double calc_centroid(std::string name, double value);
        void check_course_line(point &point, float range = 0.05);

        void concat(std::vector<geometry_msgs::Point> &v1, std::vector<geometry_msgs::Point> &v2){
            v1.insert(v1.begin(), v2.begin(), v2.end());
        }

        void clamp(double &v, double low, double high){
             if(v < low){
                 v = low;
             }else if(v > high){
                 v = high;
             }
         }
};

#endif