#ifndef TYPE_H_INCLUDED
#define TYPE_H_INCLUDED

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <cmath>

typedef enum{
    COURT_MODE_RED = 0x00,
    COURT_MODE_BLUE = 0x01,
    COURT_MODE_RED_MT = 0x02,
    COURT_MODE_BLUE_MT = 0x03
}court_mode_t;

struct point{
    double x;
    double y;
    double angle;

    point(double x = 0, double y = 0, double angle = 0) : x(x), y(y), angle(angle){}
    point(tf::Transform tp) {
        x = tp.getOrigin().getX();
        y = tp.getOrigin().getY();
        double roll, pitch, yaw;
        tf::Matrix3x3(tp.getRotation()).getRPY(roll, pitch, yaw);
        angle = -yaw;
    }

    ///原点からの距離
    double get_distance(){
        return std::sqrt(x * x + y * y);
    }

    ///二点間の距離
    double get_distance(point p){
        return std::sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
    }

    geometry_msgs::Point get_geometry_point(){
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = angle;
        return p;
    }

    tf::Transform get_tf_transform(){
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, -angle);
        transform.setRotation(q);
        return transform;
    }
};

///極座標
class polar_point{
    public:
    double rho;
    double theta;

    polar_point(double rho_ = 0, double theta_ = 0){
        ///正規化
        if(std::abs(theta_) > M_PI_2){
            theta_ = (theta_ > 0 ? theta_ - M_PI : theta_ + M_PI);
            rho_ = -rho_;
        }

        rho = rho_;
        theta = theta_;
    }

    point get_point(){
        return point(rho * cos(theta) , -rho * sin(theta));
    }

    double get_positive_normalization_theta(){
        if(theta < 0){
            return M_PI + theta;
        }
        return theta;
    }

    double get_positive_normalization_rho(){
        if(theta < 0){
            return -rho;
        }
        return rho;
    }

    bool is_equal(polar_point pp, double rho_th = 0.05, double theta_th = 0.00001){
        double rho_ = rho, theta_ = theta;
        if(theta_ > 0 && pp.theta < 0){
            pp.theta += M_PI;
            pp.rho = -pp.rho;
        }else if(theta_ < 0 && pp.theta > 0){
            theta_ += M_PI;
            rho_ = -rho;
        }
        return is_equal_number(rho_, pp.rho, rho_th) && is_equal_number(theta_, pp.theta, theta_th);
    }

    private:
        bool is_equal_number(double n1, double n2, double threshold = 0.000001){
            if(std::abs(n1 - n2) <  threshold){
                return true;
            }
            return false;
        }
};

///y = ax + bの形で保存
class line{
    private:
        double a_;
        double b_;
        polar_point pp_;
        point p1_;
        point p2_;

        bool point_line_flag; ///ポイントでインスタンスを生成したときにそのポイントのマーカー用のポイントコンテナを返す

    public:
        ///コンストラクタ
        line(double rho, double theta) : pp_(polar_point(rho, theta)) {
            rho = pp_.rho;
            theta = pp_.theta;

            if(is_equal_number(std::tan(theta), 0)){
                a_ = std::pow(10, 12);
                b_ = (rho > 0 ? -1 : 1) * std::pow(10, 12);
            }else {
                a_ = -(1.0 / std::tan(theta));
                b_ = rho * (std::sin(theta) + (std::cos(theta) / std::tan(theta)));
            }

            point_line_flag = false;
        }

        line(polar_point pp) : pp_(pp){
            double rho = pp.rho;
            double theta = pp.theta;
            if(is_equal_number(std::tan(theta), 0)){
                a_ = std::pow(10, 12);
                b_ = (rho > 0 ? -1 : 1) * std::pow(10, 12);
            }else {
                a_ = -(1.0 / std::tan(theta));
                b_ = rho * (std::sin(theta) + (std::cos(theta) / std::tan(theta)));
            }

            point_line_flag = false;
        }

        line(point p1, point p2) : p1_(p1), p2_(p2){
            if(is_equal_number(p1.x, p2.x)){ ///傾きが無限大の時
                a_ = std::pow(10, 12);
            }else{
                a_ = (p2.y - p1.y) / (p2.x - p1.x);
            }

            b_ = p1.y - a_ * p1.x;

            pp_ = calc_polar_point(point(0,0));

            point_line_flag = true;
        }

        line(){
            a_ = 0;
            b_ = 0;

            point_line_flag = false;
        }

        ///ゲッター
        double get_a(){ return a_; }
        double get_b(){ return b_; }
        polar_point get_polar_point(){ return pp_; }
        point get_p1() { return p1_; }
        point get_p2() { return p2_; }

        point get_intersection(line l){
            double theta1 = -pp_.theta;
            double theta2 = -l.get_polar_point().theta;
            double a, b, c, d;
            a = -(1.0 / std::tan(theta1));
            b = pp_.rho * (std::sin(theta1) + ((std::cos(theta1) * std::cos(theta1)) / std::sin(theta1)));
            c = -(1.0 / std::tan(theta2));
            d = l.get_polar_point().rho * (std::sin(theta2) + ((std::cos(theta2) * std::cos(theta2)) / std::sin(theta2)));

            return point((d - b) / (a - c), (a * d - b * c) / (a - c), 0);
        }

        ///visualization_msgs用のコンテナ作成
        ///中心はget_pointの位置
        std::vector<geometry_msgs::Point> get_marker_point(double length = 1.0){
            std::vector<geometry_msgs::Point> ret;

            if(point_line_flag){
                ret.push_back(p1_.get_geometry_point());
                ret.push_back(p2_.get_geometry_point());
            }else{
                length = length / 2;
                geometry_msgs::Point gp = pp_.get_point().get_geometry_point();
                gp.x += length * sin(pp_.theta);
                gp.y += length * cos(pp_.theta);
                ret.push_back(gp);

                gp = pp_.get_point().get_geometry_point();
                gp.x -= length * sin(pp_.theta);
                gp.y -= length * cos(pp_.theta);                
                ret.push_back(gp);
            }

            return ret;
        }

        ///pを基準とした極座標表示の値計算
        polar_point calc_polar_point(point p){
            double rho = std::abs(a_ * p.x - p.y + b_) / std::sqrt((a_ * a_) + 1);

            double theta;
            if(is_equal_number(a_, 0)){
                theta =  (p.y > b_ ? M_PI_2 : -M_PI_2) - p.angle;
            }else{
                theta = std::atan((1.0 / a_)) - p.angle; ///control_id:2でやるときは＋
            }

            return polar_point(rho, theta);
        }  

    private:
        bool is_equal_number(double n1, double n2, double threshold = 0.000001){
            if(std::abs(n1 - n2) <  threshold){
                return true;
            }
            return false;
        }
};

#endif