#ifndef DETECTOR_H_INCLUDED
#define DETECTOR_H_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "type.h"
#include "Hough.h"

#include <vector>
#include <boost/foreach.hpp>

#include <unordered_map>

typedef std::unordered_map<std::string, line> line_map;

class detector{
    private:
        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        visualization_msgs::Marker line_marker_;
        visualization_msgs::Marker point_marker_;

        std::string name_;
        line_map *field_line_;
        line_map *course_line_;

    public:
        detector(std::string name, line_map *field_line, line_map *course_line) : name_(name), field_line_(field_line), course_line_(course_line){
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>(name + "_marker", 10);
            
            line_marker_.header.frame_id = "base_footprint";
            line_marker_.action = visualization_msgs::Marker::ADD;
            line_marker_.type = visualization_msgs::Marker::LINE_LIST;
            line_marker_.scale.x = 0.01;
            line_marker_.color.a = 1.0;
            line_marker_.lifetime = ros::Duration(0.5);

            point_marker_.header.frame_id = "base_footprint";
            point_marker_.action = visualization_msgs::Marker::ADD;
            point_marker_.type = visualization_msgs::Marker::POINTS;
            point_marker_.scale.x = 0.05;
            point_marker_.scale.y = 0.05;
            point_marker_.color.a = 1.0;
            point_marker_.lifetime = ros::Duration(0.5);
        }

        virtual bool detect(const Hough &, point, point*) = 0;

    protected:
        line get_field_line(std::string name){
            return (*field_line_)[name];
        }

        void publish_line_marker(uint32_t id, line l, double r = 1.0f, double g = 1.0f, double b = 1.0f, double length = 1.0){
            line_marker_.header.stamp = ros::Time::now();
            line_marker_.color.r = r;
            line_marker_.color.g = g;
            line_marker_.color.b = b;
            line_marker_.color.a = 1.0;

            line_marker_.id = id;
            line_marker_.points = l.get_marker_point(length);
            
            marker_pub_.publish(line_marker_);
        }

        void publish_point_marker(uint32_t id, point p, double r = 1.0f, double g = 1.0f, double b = 1.0f){
            point_marker_.header.stamp = ros::Time::now();
            point_marker_.color.r = r;
            point_marker_.color.g = g;
            point_marker_.color.b = b;
            point_marker_.color.a = 1.0;

            point_marker_.id = id;
            point_marker_.points.clear();
            point_marker_.points.push_back(p.get_geometry_point());
            
            marker_pub_.publish(point_marker_);
        }

        static bool is_equal_number(double n1, double n2, double threshold = 0.000001){
            if(std::abs(n1 - n2) <  threshold){
                return true;
            }
            return false;
        }

};

class first_wall_detector : public detector{
    public:
        first_wall_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class third_parallel_detector : public detector{
    public:
        third_parallel_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class step_forward_detector : public detector{
    public:
        step_forward_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class step_detector : public detector{
    public:
        step_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class step_over_detector : public detector{
    public:
        step_over_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class end_parallel_detector : public detector{
    public:
        end_parallel_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class end_parallel_detector2 : public detector{
    public:
        end_parallel_detector2(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

class mt_inner_detector : public detector{
    public:
        mt_inner_detector(std::string name, line_map *field_line, line_map *course_line) : detector(name, field_line, course_line){}
        bool detect(const Hough &, point, point*);
};

#endif