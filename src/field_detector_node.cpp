#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <mutex>

#include "type.h"
#include "Hough.h"
#include "detector_manager.h"

class field_detector{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;

    std::mutex m_;

    line_detector_manager *line_detector_;

    public:
    field_detector(){
        ros::NodeHandle pn("~");

        pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/field_plane", 1, &field_detector::callback, this);

        line_detector_ = new line_detector_manager();
    }

    
    void callback(sensor_msgs::PointCloud2 pc2){
        std::lock_guard<std::mutex> lock(m_);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (pc2, cloud);

        ros::WallTime ros_begin  = ros::WallTime::now();

        std::vector<point> points;
        points.resize(cloud.points.size());
        // ROS_INFO_STREAM("point num " << cloud.points.size());s
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            points[i] = point(cloud.points[i].x, cloud.points[i].y);
        }

        ///ハフ変換クラスのインスタンス生成
        Hough hough(points);


        ///自己位置更新処理
        line_detector_->update(hough);

        // ros::WallDuration ros_duration  = ros::WallTime::now()-ros_begin;
        // ROS_INFO_STREAM("proccesing time : " << ros_duration.nsec / 1000000.0 << " [ms]");
    }
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "field_detector");

    field_detector f;

    ros::spin();
}
