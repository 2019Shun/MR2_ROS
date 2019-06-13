#include "detector_manager.h"

///コンストラクタ
line_detector_manager::line_detector_manager(){
    field_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/field_marker", 10);
    bno_angle_sub_ = nh_.subscribe<std_msgs::Float32>("/nucleo/bno_angle", 1, &line_detector_manager::bno_angle_cb, this);

    court_mode_srv_server_ = nh_ .advertiseService("set_court_mode", &line_detector_manager::set_court_mode, this);
    initial_point_srv_server_ = nh_ .advertiseService("set_initial_point", &line_detector_manager::set_initial_point, this);

    ///Detectorインスタンス生成
    first_wall_ = new first_wall_detector("initial_wall", &field_line_, &course_line_);
    third_parallel_ = new third_parallel_detector("third_parallel_wall", &field_line_, &course_line_);
    step_forward_ = new step_forward_detector("step_forward_wall", &field_line_, &course_line_);
    step_over_ = new step_over_detector("step_over_wall", &field_line_, &course_line_);
    step_ = new step_detector("step_wall", &field_line_, &course_line_);
    end_parallel_ = new end_parallel_detector("end_parallel", &field_line_, &course_line_);
    mt_inner_ = new mt_inner_detector("mt_inner", &field_line_, &course_line_);

    bno_angle_ = 0;
}

///現在位置とNucleoから得た角度より適切な検出クラスを選んで自己位置を計算
void line_detector_manager::update(const Hough &hough){
    point cur_point = point_; ///現在位置をコピー
    point p = point_;
    point p0; /// 値取得用

    int control_id = 0;

    switch (court_mode_)
    {
        case COURT_MODE_RED:
        case COURT_MODE_BLUE:
            if(court_mode_ == COURT_MODE_RED){
                if(cur_point.x < 2.5){
                    control_id = 1;
                }else if(cur_point.x < 3.2){
                    control_id = 2;
                }else if(cur_point.x < 5.5 && cur_point.y < 0.4 ){
                    control_id = 3;
                }else{
                    control_id = 4;
                }
            }else{
                if(cur_point.x < 2.5){
                    control_id = 1;
                }else if(cur_point.x < 3.2){
                    control_id = 2;
                }else if(cur_point.x < 5.5 && cur_point.y > -0.4 ){
                    control_id = 3;
                }else{
                    control_id = 4;
                }
            }

            switch (control_id)
            {
                case 1: /// スタートから段差の手前まで
                    if(!first_wall_->detect(hough, cur_point, &p0)){
                        return;
                    }
                    break;

                case 2: ///
                    // if(!step_forward_->detect(hough, cur_point, &p0)){
                    //     return;
                    // }
                    if(!step_over_->detect(hough, cur_point, &p0)){
                        return;
                    }
                    break;

                case 3: /// 
                    if(!step_over_->detect(hough, cur_point, &p0)){
                        return;
                    }
                    break;

                case 4:
                    if(!end_parallel_->detect(hough, cur_point, &p0)){
                        return;
                    }
                    break;

                default:
                    return;
            }
            /// 指定したコースラインから離れすぎた場合への対策
            check_course_line(p0, 0.50); 
            break;

        case COURT_MODE_RED_MT:
        case COURT_MODE_BLUE_MT:
            if(!mt_inner_->detect(hough, cur_point, &p0)){
                return;
            }
            /// 指定したコースラインから離れすぎた場合への対策
            check_course_line(p0, 0.20); 
            break;

        default:
            return;
    }
    
    /// ジャイロセンサから取得した値でクランプ
    clamp(p0.angle, bno_angle_ - 0.2, bno_angle_ + 0.2);

    /// ROS_INFO_STREAM("dist " << point_.get_distance(p));
    /// 移動幅が500mm以上の時は外れ値とする
    if(point_.get_distance(p0) < 0.5){
        if(control_id == 4){
            // lpf("x4", p0.x, 0.6);
            // lpf("y4", p0.y, 0.6);
            // lpf("angle4", p0.angle, 0.5);
        }
    }else{
        return;
    }

    // p.x = lpf("x", p0.x, 0.6);
    // p.y = lpf("y", p0.y, 0.6);
    // p.angle = lpf("angle", p0.angle, 0.5);
    p = p0;
    p.angle = bno_angle_; //test

    point_ = p;
    point_broadcaster();
    ROS_INFO_STREAM("Point: " << point_.x << " , " << point_.y << " , " << point_.angle << " , " << bno_angle_);

    ///デバッグ用．なくても良い
    field_line_create(false);
}

///フィールドの各ラインを生成し，連想配列に格納
void line_detector_manager::field_line_create(bool line_update_flag){
    if(line_update_flag) {
        field_line_.clear();
        course_line_.clear();
        switch (court_mode_)
        {
            case COURT_MODE_RED:
                field_line_["start"] = line(point(0, 0), point(0, -1.9));
                field_line_["first_outer"] = line(point(0, 0), point(3.5, 0));
                field_line_["first_inner"] = line(point(0, -1.9), point(4.018, -1.9));
                field_line_["outer_oblique"] = line(point(3.5, 0), point(3.95, 0.45));
                field_line_["inner_oblique"] = line(point(4.018, -1.9), point(5.45, -0.468));
                field_line_["step_forward"] = line(point(3.57071, 0.07071), point(4.77986, -1.113844));
                field_line_["step_backward"] = line(point(3.78284, 0.28284), point(4.992, -0.92631));
                field_line_["first_parallel"] = line(point(3.95, 0.45), point(3.95, 2.0));
                field_line_["second_parallel"] = line(point(5.45, -0.468), point(5.45, 0.55));
                field_line_["third_parallel"] = line(point(6.45, -2.15), point(6.45, 0.55));
                field_line_["second_outer"] = line(point(3.95, 2.0), point(7.95, 2.0));
                field_line_["second_inner"] = line(point(5.45, 0.55), point(6.45, 0.55));
                field_line_["end_parallel"] = line(point(7.95, 2.0), point(7.95, -2.15));

                {
                    point tp_start(0.5, -0.7);
                    point tp_before_step(3.3, -0.7);
                    point tp_after_step(5.0, 1.3);
                    point tp_goal(7.3, 1.3);
                    course_line_["first"] = line(tp_start, tp_before_step);
                    course_line_["second"] = line(tp_before_step, tp_after_step);
                    course_line_["third"] = line(tp_after_step, tp_goal);
                }
                break;

            case COURT_MODE_BLUE:
                field_line_["start"] = line(point(0, 0), point(0, 1.9));
                field_line_["first_outer"] = line(point(0, 0), point(3.5, 0));
                field_line_["first_inner"] = line(point(0, 1.9), point(4.018, 1.9));
                field_line_["outer_oblique"] = line(point(3.5, 0), point(3.95, -0.45));
                field_line_["inner_oblique"] = line(point(4.018, 1.9), point(5.45, 0.468));
                field_line_["step_forward"] = line(point(3.57071, -0.07071), point(4.77986, 1.113844));
                field_line_["step_backward"] = line(point(3.78284, -0.28284), point(4.992, 0.92631));
                field_line_["first_parallel"] = line(point(3.95, -0.45), point(3.95, -2.0));
                field_line_["second_parallel"] = line(point(5.45, 0.468), point(5.45, -0.55));
                field_line_["third_parallel"] = line(point(6.45, 2.15), point(6.45, -0.55));
                field_line_["second_outer"] = line(point(3.95, -2.0), point(7.95, -2.0));
                field_line_["second_inner"] = line(point(5.45, -0.55), point(6.45, -0.55));
                field_line_["end_parallel"] = line(point(7.95, -2.0), point(7.95, 2.15));

                {
                    point tp_start(0.5, 0.7);
                    point tp_before_step(3.3, 0.7);
                    point tp_after_step(5.0, -1.3);
                    point tp_goal(7.3, -1.3);
                    course_line_["first"] = line(tp_start, tp_before_step);
                    course_line_["second"] = line(tp_before_step, tp_after_step);
                    course_line_["third"] = line(tp_after_step, tp_goal);
                }
                break;

            case COURT_MODE_RED_MT:
                field_line_["line_inner"] = line(point(1.45, -1.45), point(4.2, -1.45));
                field_line_["line_outer"] = line(point(0, 0), point(4.2, 0));
                field_line_["line_end"] = line(point(4.2, -1.45), point(4.2, 0));
                {
                    point tp_start(0.5, -0.7);
                    point tp_goal(4.0, -0.7);
                    course_line_["target"] = line(tp_start, tp_goal);
                }
                break;

            case COURT_MODE_BLUE_MT:
                field_line_["line_inner"] = line(point(1.45, 1.45), point(4.2, 1.45));
                field_line_["line_outer"] = line(point(0, 0), point(4.2, 0));
                field_line_["line_end"] = line(point(4.2, 1.45), point(4.2, 0));
                {
                    point tp_start(0.5, 0.7);
                    point tp_goal(4.0, 0.7);
                    course_line_["target"] = line(tp_start, tp_goal);
                }
                break;
        }
    }

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.01;
    line_marker.color.g = 1.0;
    line_marker.color.a = 1.0;
    line_marker.lifetime = ros::Duration(0.5);

    typedef std::map<std::string, line>::const_reference type;
    uint32_t id = 0;
    BOOST_FOREACH (type line, field_line_) {
        line_marker.id = id++;
        line_marker.ns = line.first;
        line_marker.points = field_line_[line.first].get_marker_point();
        field_marker_pub_.publish(line_marker);
    }

    line_marker.color.g = 0.0;
    line_marker.color.r = 1.0;
    BOOST_FOREACH (type line, course_line_) {
        line_marker.id = id++;
        line_marker.ns = "course_" + line.first;
        line_marker.points = course_line_[line.first].get_marker_point();
        field_marker_pub_.publish(line_marker);
    }
}

///値にローパスフィルターをかける
///名前を指定して連想配列で管理
double line_detector_manager::lpf(std::string name, double &value, double a){
    static std::unordered_map<std::string, double> prev_value;
    
    ///内部配列の初期化
    if(name == "internal_initialize"){
        prev_value.clear();
        return 0;
    }

    if(prev_value.count(name) == 0){
        ///Initial value
        prev_value[name] = value;
        return value;
    }

    value =  prev_value[name] * a + value * (1.0 - a);
    prev_value[name] = value;
    return value;
}

///重心を計算．今のところ使ってない??
double line_detector_manager::calc_centroid(std::string name, double value){
    static std::unordered_map<std::string, std::deque<double>> value_map;
    if(value_map.count(name) == 0){
        ///Initial value
        value_map[name].push_back(value);
    }else{
        if(value_map[name].size() >= 5){
            value_map[name].pop_front();
        }
        value_map[name].push_back(value);
    }
    
    double sum = 0;
    for(auto itr = value_map[name].begin(); itr != value_map[name].end(); ++itr) {
        sum += (*itr);
    }

    return sum / value_map[name].size();
}


/// コースラインと現在地点より理想的な現在地を取得
void line_detector_manager::check_course_line(point &point, float range){
    double y;
    switch (court_mode_)
    {
        case COURT_MODE_RED:
        case COURT_MODE_BLUE:
            if(point.x < course_line_["first"].get_p2().x){
                y = course_line_["first"].get_p2().y;
            }else if(point.x < course_line_["second"].get_p2().x){
                y = course_line_["second"].get_p1().y + (court_mode_ == COURT_MODE_BLUE ? -1:1) * (point.x - course_line_["second"].get_p1().x);
            }else if(point.x < course_line_["third"].get_p2().x){
                y = course_line_["third"].get_p2().y;
            }else{
                return;
            }

            if(std::abs(point.y - y) < range){
                return;
            }
            if(point.y - y > 0){
                point.y = y + range;
            }else{
                point.y = y - range;
            }
            break;
    
        case COURT_MODE_RED_MT:
        case COURT_MODE_BLUE_MT:
            y = course_line_["target"].get_p2().y;
            if(std::abs(point.y - y) < range){
                return;
            }
            if(point.y - y > 0){
                point.y = y + range;
            }else{
                point.y = y - range;
            }
            break;

        default:
            break;
    }
}