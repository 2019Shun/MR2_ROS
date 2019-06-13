#include "detector.h"

bool first_wall_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point third_parallel_pp = get_field_line("third_parallel").calc_polar_point(current_pos);
    polar_point inner_pp = get_field_line("first_inner").calc_polar_point(current_pos);
    point target_point = line(third_parallel_pp).get_intersection(line(inner_pp));
    point p;

    std::vector<line> third_parallel_predict = hough.calc_hough(third_parallel_pp, 0.2, 0.2, 1);
    std::vector<line> inner_predict = hough.calc_hough(inner_pp, 0.3, 0.2, 1);

    bool detect_flag0 = false;
    if(third_parallel_predict.size() > 0){
        publish_line_marker(0, third_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(third_parallel_predict.size() > 0){
        if(inner_predict.size() > 0){  
            double a = third_parallel_predict[0].get_polar_point().theta;
            double b = inner_predict[0].get_polar_point().theta;

            // ROS_INFO_STREAM("first " << inner_predict[0].get_polar_point().get_positive_normalization_rho());
            if(is_equal_number(std::abs(b - a), M_PI_2, 0.01)){
                p = inner_predict[0].get_intersection(third_parallel_predict[0]);
                publish_point_marker(2, p, 1.0, 0.0, 0.0);
                publish_line_marker(1, inner_predict[0], 1.0 , 0, 0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    if(detect_flag0){
        out->x = get_field_line("third_parallel").get_polar_point().rho - third_parallel_predict[0].get_polar_point().rho;
        if(detect_flag1){
            double side = inner_predict[0].get_polar_point().get_positive_normalization_rho();
            out->y = (side > 0 ? -1 : 1) * 1.9 + side;
        }else{
            out->y = current_pos.y;
        }
        out->angle = -third_parallel_predict[0].get_polar_point().theta;
        return true;
    }else{
        return false;
    }
}

bool third_parallel_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point third_parallel_pp = get_field_line("third_parallel").calc_polar_point(current_pos);

    std::vector<line> third_parallel_predict = hough.calc_hough(third_parallel_pp, 0.2, 0.2, 1);


    bool detect_flag = false;
    if(third_parallel_predict.size() > 0){
        publish_line_marker(0, third_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag = true;
    }

    if(detect_flag){
        out->x = get_field_line("third_parallel").get_polar_point().rho - third_parallel_predict[0].get_polar_point().rho;
        out->y = current_pos.y;
        out->angle = third_parallel_predict[0].get_polar_point().theta;
        return true;
    }else{
        return false;
    }
}

bool step_forward_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point third_parallel_pp = get_field_line("third_parallel").calc_polar_point(current_pos);
    polar_point inner_oblique_pp = get_field_line("inner_oblique").calc_polar_point(current_pos);
    point target_point = line(third_parallel_pp).get_intersection(line(inner_oblique_pp));
    point p;

    std::vector<line> third_parallel_predict = hough.calc_hough(third_parallel_pp, 0.2, 0.2, 1);
    std::vector<line> inner_oblique_predict = hough.calc_hough(inner_oblique_pp, 0.4, 0.2, 1);

    bool detect_flag0 = false;
    if(third_parallel_predict.size() > 0){
        publish_line_marker(0, third_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(third_parallel_predict.size() > 0){
        if(inner_oblique_predict.size() > 0){  
            double a = third_parallel_predict[0].get_polar_point().theta;
            double b = inner_oblique_predict[0].get_polar_point().theta;
            
            // ROS_INFO_STREAM("step a " << a << " , b" << b << " , diff" << std::abs(b - a));
            
            if(is_equal_number(std::abs(b - a), M_PI_4, 0.03)){
                p = inner_oblique_predict[0].get_intersection(third_parallel_predict[0]);
                publish_point_marker(2, p, 1.0, 0.0, 0.0);
                publish_line_marker(1, inner_oblique_predict[0], 1.0 , 0, 0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    if(detect_flag0){
        out->x = get_field_line("third_parallel").get_polar_point().rho - third_parallel_predict[0].get_polar_point().rho;
        if(detect_flag1){
            out->y = current_pos.y + (target_point.y - p.y);
        }else{
            out->y = current_pos.y;
        }
        out->angle = -third_parallel_predict[0].get_polar_point().theta;
        return true;
    }else{
        return false;
    }
}

bool step_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point third_parallel_pp = get_field_line("third_parallel").calc_polar_point(current_pos);
    polar_point inner_oblique_pp = get_field_line("inner_oblique").calc_polar_point(current_pos);
    polar_point second_outer_pp = get_field_line("second_outer").calc_polar_point(current_pos);
    point target_point = line(third_parallel_pp).get_intersection(line(inner_oblique_pp));
    point target_point2 = line(third_parallel_pp).get_intersection(line(second_outer_pp));
    point p1, p2;

    std::vector<line> third_parallel_predict = hough.calc_hough(third_parallel_pp, 0.2, 0.2, 1);
    std::vector<line> inner_oblique_predict = hough.calc_hough(inner_oblique_pp, 0.2, 0.2, 1);
    std::vector<line> second_outer_predict = hough.calc_hough(second_outer_pp, 0.2, 0.1, 1);

    bool detect_flag0 = false;
    if(third_parallel_predict.size() > 0){
        publish_line_marker(0, third_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(third_parallel_predict.size() > 0){
        if(inner_oblique_predict.size() > 0){  
            double a = third_parallel_predict[0].get_polar_point().theta;
            double b = inner_oblique_predict[0].get_polar_point().theta;

            if(is_equal_number(std::abs(b - a), M_PI_4, 0.01)){
                p1 = inner_oblique_predict[0].get_intersection(third_parallel_predict[0]);
                publish_point_marker(2, p1, 1.0, 0.0, 0.0);
                publish_line_marker(1, inner_oblique_predict[0], 1.0 , 0, 0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    bool detect_flag2 = false;
    if(third_parallel_predict.size() > 0){
        if(second_outer_predict.size() > 0){  
            double a = third_parallel_predict[0].get_polar_point().theta;
            double b = second_outer_predict[0].get_polar_point().theta;

            if(is_equal_number(std::abs(b - a), M_PI_2, 0.01)){
                p2 = second_outer_predict[0].get_intersection(third_parallel_predict[0]);
                publish_point_marker(2, p2, 1.0, 0.0, 0.0);
                publish_line_marker(1, second_outer_predict[0], 1.0 , 0, 0.0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag2 = true;
            }
        }
    }

    if(detect_flag0){
        out->x = get_field_line("third_parallel").get_polar_point().rho - third_parallel_predict[0].get_polar_point().rho;
        if(detect_flag1){
            out->y = current_pos.y + (target_point.y - p1.y);
            // out->y = std::abs(second_outer_predict[0].get_polar_point().rho) - 1.9;
        }else if(detect_flag2 && current_pos.x > 3.8){
            out->y = current_pos.y + (second_outer_predict[0].get_polar_point().rho - second_outer_pp.rho);
            // out->y = current_pos.y + (target_point2.y - p2.y);
        }else{
            out->y = current_pos.y;
        }
        out->angle = -third_parallel_predict[0].get_polar_point().theta;
        return true;
    }else{
        return false;
    }
}

bool step_over_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point third_parallel_pp = get_field_line("third_parallel").calc_polar_point(current_pos);
    polar_point second_outer_pp = get_field_line("second_outer").calc_polar_point(current_pos);
    point target_point = line(third_parallel_pp).get_intersection(line(second_outer_pp));
    point p;

    std::vector<line> third_parallel_predict = hough.calc_hough(third_parallel_pp, 0.2, 0.2, 1);
    std::vector<line> second_outer_predict = hough.calc_hough(second_outer_pp, 0.4, 0.2, 1);

    bool detect_flag0 = false;
    if(third_parallel_predict.size() > 0){
        publish_line_marker(0, third_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(third_parallel_predict.size() > 0){
        if(second_outer_predict.size() > 0){  
            double a = third_parallel_predict[0].get_polar_point().theta;
            double b = second_outer_predict[0].get_polar_point().theta;

            if(is_equal_number(std::abs(b - a), M_PI_2, 0.03)){
                p = second_outer_predict[0].get_intersection(third_parallel_predict[0]);
                publish_point_marker(2, p, 1.0, 0.0, 0.0);
                publish_line_marker(1, second_outer_predict[0], 1.0 , 0, 0.0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    if(detect_flag0){
        out->x = get_field_line("third_parallel").get_polar_point().rho - third_parallel_predict[0].get_polar_point().rho;
        if(detect_flag1){
            double side = (second_outer_predict[0].get_polar_point().rho - second_outer_pp.rho);
            out->y = current_pos.y + (get_field_line("second_outer").get_p1().y > 0 ? -1 : 1) * side;
        }else{
            out->y = current_pos.y;
        }
        out->angle = -third_parallel_predict[0].get_polar_point().theta;
        return true;
    }else{
        return false;
    }
}

bool end_parallel_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point second_outer_pp = get_field_line("second_outer").calc_polar_point(current_pos);
    polar_point end_parallel_pp = get_field_line("end_parallel").calc_polar_point(current_pos);
    point target_point = line(end_parallel_pp).get_intersection(line(second_outer_pp));
    point p;

    std::vector<line> second_outer_predict = hough.calc_hough(second_outer_pp, 0.2, 0.2, 1);
    std::vector<line> end_parallel_predict = hough.calc_hough(end_parallel_pp, 0.2, 0.2, 1);

    bool detect_flag0 = false;
    if(end_parallel_predict.size() > 0){
        publish_line_marker(0, end_parallel_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(detect_flag0){
        if(second_outer_predict.size() > 0){ 
            double a = second_outer_predict[0].get_polar_point().get_positive_normalization_theta();
            double b = end_parallel_predict[0].get_polar_point().get_positive_normalization_theta();

            if(is_equal_number(std::abs(b - a), M_PI_2, 0.1)){
                p = second_outer_predict[0].get_intersection(end_parallel_predict[0]);
                publish_point_marker(2, p, 1.0, 1.0, 1.0);
                publish_line_marker(1, second_outer_predict[0], 1.0 , 0, 0.0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    if(detect_flag0){
        out->x = get_field_line("end_parallel").get_polar_point().rho - end_parallel_predict[0].get_polar_point().rho;
        out->angle = -end_parallel_predict[0].get_polar_point().theta;
        // ROS_WARN_STREAM("end " << out->x << ", " << out->angle);
        if(detect_flag1){
            out->y = current_pos.y + (second_outer_predict[0].get_polar_point().get_positive_normalization_rho() - second_outer_pp.get_positive_normalization_rho());
        }else{
            out->y = current_pos.y;
        }
        return true;
    }else{
        return false;
    }
}

bool end_parallel_detector2::detect(const Hough &hough, point current_pos, point *out){
    polar_point second_outer_pp = get_field_line("second_outer").calc_polar_point(current_pos);
    polar_point end_parallel_pp = get_field_line("end_parallel").calc_polar_point(current_pos);
    point target_point = line(end_parallel_pp).get_intersection(line(second_outer_pp));
    point p;

    std::vector<line> second_outer_predict = hough.calc_hough(second_outer_pp, 0.2, 0.1, 1);
    std::vector<line> end_parallel_predict = hough.calc_hough(end_parallel_pp, 0.4, 0.6, 1);

    bool detect_flag0 = false;
    if(second_outer_predict.size() > 0){
        publish_line_marker(0, second_outer_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    bool detect_flag1 = false;
    if(detect_flag0){
        if(end_parallel_predict.size() > 0){ 
            double a = second_outer_predict[0].get_polar_point().get_positive_normalization_theta();
            double b = end_parallel_predict[0].get_polar_point().get_positive_normalization_theta();

            // ROS_INFO_STREAM("end0 " << a << " , " << b);

            if(is_equal_number(std::abs(b - a), M_PI_2, 0.1)){
                p = second_outer_predict[0].get_intersection(end_parallel_predict[0]);
                publish_point_marker(2, p, 1.0, 1.0, 1.0);
                publish_line_marker(1, end_parallel_predict[0], 1.0 , 0, 0.0, 1.0);
                publish_point_marker(3, target_point, 1.0, 1.0, 1.0);
                detect_flag1 = true;
            }
        }
    }

    if(detect_flag0){
        out->y = current_pos.y + (second_outer_predict[0].get_polar_point().get_positive_normalization_rho() - second_outer_pp.get_positive_normalization_rho());
        // ROS_INFO_STREAM("end1 " << second_outer_predict[0].get_polar_point().get_positive_normalization_rho() << " , " << second_outer_pp.get_positive_normalization_rho());
        if(detect_flag1){
            out->x = get_field_line("end_parallel").get_polar_point().rho - end_parallel_predict[0].get_polar_point().rho;
            // ROS_WARN_STREAM("end " << out->x);
        }else{
            out->x = current_pos.x;
        }
        out->angle = -(second_outer_predict[0].get_polar_point().get_positive_normalization_theta() - M_PI_2);
        return true;
    }else{
        return false;
    }
}

bool mt_inner_detector::detect(const Hough &hough, point current_pos, point *out){
    polar_point inner_pp = get_field_line("line_inner").calc_polar_point(current_pos);
    point p;

    std::vector<line> inner_predict = hough.calc_hough(inner_pp, 0.2, 0.2, 1);

    bool detect_flag0 = false;
    if(inner_predict.size() > 0){
        publish_line_marker(0, inner_predict[0], 1.0 , 0, 0, 1.0);
        detect_flag0 = true;
    }

    if(detect_flag0){
        out->x = current_pos.x;
        out->y = 1.45 + inner_predict[0].get_polar_point().get_positive_normalization_rho();
        out->angle = (M_PI_2 - inner_predict[0].get_polar_point().get_positive_normalization_theta());
        // ROS_WARN_STREAM(out->y << " , " << out->angle);
        return true;
    }else{
        return false;
    }
}