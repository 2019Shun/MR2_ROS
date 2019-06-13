#ifndef HOUGH_H_INCLUDED
#define HOUGH_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <cmath>
#include <vector>

#include <type.h>

class Hough{
    private:
        std::vector<point> points_;
    
    public:
        Hough(std::vector<point> points){
            points_ = points;
        }

        std::vector<line> calc_hough(double rho_min, double rho_max, double rho_width, double theta_min, double theta_max, double theta_width, int num) const{
            int rho_num = (rho_max - rho_min) / rho_width;
            int theta_num = (theta_max - theta_min) / theta_width;

            std::vector<std::vector<int>> counter;///コンテナ生成と0初期化
            counter.resize(theta_num);
            for( int i = 0; i < theta_num; i++ ){
                counter[i].resize(rho_num);
            }

            int max_i = 0, max_j = 0;
            for(auto itr = points_.begin(); itr != points_.end(); ++itr) {
                for(int i = 0; i < theta_num; i++){
                    double theta = theta_min + theta_width * i;
                    double rho = itr->x * cos(theta) - itr->y * sin(theta);
                    for(int j = 0; j < rho_num; j++){
                        if(rho_min + rho_width * j < rho && rho < rho_min + rho_width * (j + 1)){
                            // counter[i][j]++;
                            if(++counter[i][j] > counter[max_i][max_j]){
                                max_i = i;
                                max_j = j;
                            }
                            break;
                        }
                    }
                }
            }
            
            std::vector<line> ret_lines;
            if(num == 1){
                if(counter[max_i][max_j] > 2){
                    ret_lines.push_back(line(rho_min + rho_width * max_j, theta_min + theta_width * max_i));
                }
            }else{
                ///rho, thetaの格納
                std::vector<polar_point_count> lines;
                for(int i = 0; i < theta_num; i++){
                    for(int j = 0; j < rho_num; j++){
                        polar_point_count l(rho_min + rho_width * j, theta_min + theta_width * i, counter[i][j]);
                        lines.push_back(l);
                    }
                }

                ///vectorをcompareLineとstd::sortを使ってソートする．
                std::sort(lines.begin(), lines.end());

                ///指定された個数だけ返す
                for(int i = 0; i < (num < lines.size() ? num : lines.size()); i++){
                    if(lines.back().get_count() == 0){
                        break;
                    }
                    line l(lines.back().rho,lines.back().theta);
                    ret_lines.push_back(l);
                    lines.pop_back();
                }
            }

            return ret_lines;
        }

        std::vector<line> calc_hough(polar_point pp, double rho_th, double theta_th, int num) const{
            return calc_hough(pp.rho - rho_th, pp.rho + rho_th, 0.01, pp.theta - theta_th, pp.theta + theta_th, (theta_th * 2.0) / 10.0, num);
        }

        std::vector<line> calc_hough(polar_point pp, int num) const{
            return calc_hough(pp.rho - 0.4, pp.rho + 0.4, 0.01, pp.theta - 0.3, pp.theta + 0.3, 0.6 / 10.0, num);
        }

    private:
        class polar_point_count : public polar_point{
            private:
            int count_; 

            public:
            polar_point_count(double rho_ = 0, double theta_ = 0, int count = 0) : polar_point(rho_, theta_), count_(count){

            }

            int get_count(){return count_; }

            bool operator>(polar_point_count pp){
                return count_ > pp.get_count();
            }

            bool operator<(polar_point_count pp){
                return count_ < pp.get_count();
            }

            bool operator==(polar_point_count pp){
                return count_ == pp.get_count();
            }
        };
};

#endif