#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <mr2_ros/CourtMode.h>
#include <mr2_ros/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <mutex>

#include "type.h"

class field_position{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber court_mode_sub_;    //For nucleo
        ros::Subscriber initial_pose_sub_;  //For nucleo
        ros::Publisher ack_from_pc_pub_;    //For nucleo
        ros::Publisher pose_pub_;    //For nucleo
        ros::ServiceClient court_mode_srv_client_;
        ros::ServiceClient court_mode_srv_client2_;
        ros::ServiceClient initial_point_srv_client_;

        mr2_ros::CourtMode court_mode_;
        mr2_ros::Point initial_point_;

        tf::TransformListener listener_;

        std::mutex m_;

    public:
        field_position(){
            court_mode_sub_ = nh_.subscribe<std_msgs::Int8>("/nucleo/court_mode", 10, &field_position::court_mode_cb, this);
            initial_pose_sub_ = nh_.subscribe<geometry_msgs::Point>("/nucleo/initial_pose", 10, &field_position::initial_pose_cb, this);
            
            court_mode_srv_client_ = nh_.serviceClient<mr2_ros::CourtMode>("set_court_mode");
            court_mode_srv_client2_ = nh_.serviceClient<mr2_ros::CourtMode>("set_court_mode2");
            initial_point_srv_client_ = nh_.serviceClient<mr2_ros::Point>("set_initial_point");

            ack_from_pc_pub_ = nh_.advertise<std_msgs::Empty>("nucleo/ack_from_pc", 10);
            pose_pub_ = nh_.advertise<geometry_msgs::Point>("nucleo/pose", 10);
        }

        void court_mode_cb(std_msgs::Int8 court_mode){
            std::lock_guard<std::mutex> lock(m_);
            court_mode_.request.court_mode = court_mode;
            ack_for_initial_subscribe(0);
        }

        void initial_pose_cb(geometry_msgs::Point point){
            std::lock_guard<std::mutex> lock(m_); 
            initial_point_.request.point = point;
            ack_for_initial_subscribe(1);
        }

        void ack_for_initial_subscribe(int no){ //no == 0 : court_color, no == 1 : initial_point
            static float ts[2] = {0.0, 0.0};

            ts[no] = ros::Time::now().toSec();

            if(abs(ts[0] - ts[1]) > 1.0){
                return;
            }

            std_msgs::Empty empty_msg;
            ack_from_pc_pub_.publish(empty_msg);

            ROS_INFO("Send ACK to nucleo");
            ROS_INFO("Initialize mr2 position!");

            ///サービスコール
            court_mode_srv_client_.call(court_mode_);
            court_mode_srv_client2_.call(court_mode_);          
            initial_point_srv_client_.call(initial_point_);
        }
        
        void pose_publisher(){
            std::lock_guard<std::mutex> lock(m_);
            ///現在地点の取得
            tf::StampedTransform transform;
            try{
                listener_.lookupTransform("/map", "/base_footprint",  ros::Time(0), transform);
            } catch (tf::TransformException ex){
                // ROS_ERROR("%s",ex.what());
                return;
            }
            point p(transform);
            pose_pub_.publish(p.get_geometry_point());
        }

        void loop(){
            ros::AsyncSpinner spinner(1);  //spinを処理するスレッド数を引数に渡す
            spinner.start();

            ros::Rate loop(20);
            while (ros::ok()){
                pose_publisher();
                loop.sleep();
            }
            spinner.stop();
        }

};

main(int argc, char **argv)
{
    ros::init(argc, argv, "field_position_node");
    field_position fp;
    fp.loop();    
}
