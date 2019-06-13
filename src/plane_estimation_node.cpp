#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <mr2_ros/CourtMode.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "type.h"

class plane_estimation{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher plane_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher pointcloud_tmp_pub_;

    tf::TransformBroadcaster br_;
    tf::TransformListener listener_;

    //Static Parameter
    std::string pointcloud2_frame_;
    std::string m_footprint_frame_; //計測用
    std::string m_base_link_frame_; //計測用
    std::string m_lidar_link_frame_;//計測用
    std::string m_lidar_frame_;     //計測用
    std::string footprint_frame_;
    std::string base_link_frame_;
    std::string laser_frame_;
    std::string pub_name_;
    bool plane_remove_;
    double plane_threshold_;
    double sor_threshold_;
    double filter_z_min_;
    double filter_z_max_;
    double voxel_value_;

    double field_pc_x_max_;
    double field_pc_x_min_;
    double field_pc_y_max_;

    ros::ServiceServer court_mode_srv_server2_;
    court_mode_t court_mode_;

    public:
    ///コンストラクタ
    plane_estimation(){
        ros::NodeHandle pn("~");

        pointcloud2_frame_ = "/hokuyo3d/hokuyo_cloud2";
        m_footprint_frame_ = "map";
        m_base_link_frame_ = "m_base_link";
        m_lidar_link_frame_ = "m_lidar_link";
        m_lidar_frame_ = "m_lidar";

        footprint_frame_ = "base_footprint";
        base_link_frame_ = "base_link";
        laser_frame_ = "lidar";

        pn.getParam("pub_name",  pub_name_);
        pn.getParam("plane_remove",  plane_remove_);
        pn.param("plane_threshold",  plane_threshold_, 0.0);
        pn.param("sor_threshold",  sor_threshold_, 0.01);
        pn.param("filter_z_min",  filter_z_min_, 0.0);
        pn.param("filter_z_max",  filter_z_max_, 0.1);
        pn.param("voxel_value",  voxel_value_, 0.01);

        pn.param("field_pc_x_max",  field_pc_x_max_, 2.0);
        pn.param("field_pc_x_min",  field_pc_x_min_, -3.0);
        pn.param("field_pc_y_max",  field_pc_y_max_, 9.0);

        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud2_frame_, 10, &plane_estimation::callback, this);
        plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_name_, 1);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/field_pointcloud", 1);
        pointcloud_tmp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/field_pointcloud_tmp", 1);

        court_mode_srv_server2_ = nh_ .advertiseService("set_court_mode2", &plane_estimation::set_court_mode, this);
        court_mode_ = COURT_MODE_BLUE; ///デフォルト
    }

    ///方向ベクトルを向く姿勢(Quaternion)を返す.
    ///plane_normal:方向ベクトル.
    ///dir_up:方向ベクトル.どの方向を向かせるか.
    tf::Quaternion get_direction_quaternion(tf::Vector3 plane_normal, tf::Vector3 dir_up){
        tf::Vector3 dir_plane = plane_normal.cross(dir_up);
        return tf::Quaternion(dir_plane, -dir_up.angle(plane_normal)); 
    }

    ///二点間のtransformを返す
    ///それぞれのフレームは静的であることが条件
    tf::StampedTransform get_static_transform(std::string target, std::string source){
        tf::StampedTransform transform;
        transform.setIdentity();
        try{
            listener_.lookupTransform(target, source,  ros::Time(0), transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        return transform;
    }

    ///センサからのデータを受け取るコールバック関数
    void callback(sensor_msgs::PointCloud2 pc2){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
        sensor_msgs::PointCloud2 plane_pc2;
        sensor_msgs::PointCloud2 tmp_pc = pc2; 

        /// 計測した点群データの親フレームを[laser]->[laser_link]にする(正規化)
        // pcl_ros::transformPointCloud(m_lidar_link_frame_, get_static_transform(m_lidar_frame_, m_lidar_link_frame_), pc2, pc2);
        pcl_ros::transformPointCloud(m_lidar_link_frame_, get_static_transform(m_lidar_link_frame_, m_lidar_frame_), pc2, pc2);
        pcl_ros::transformPointCloud(m_lidar_link_frame_, get_static_transform(m_lidar_link_frame_, m_lidar_frame_), tmp_pc, tmp_pc);

        pre_pointcloud_process(pc2);

        pcl::fromROSMsg (pc2, cloud);

        ///ダウンサンプリング
        // ROS_INFO_STREAM("Before voxel " << cloud.size());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
        voxelgrid_filter.setInputCloud(cloud.makeShared());
        voxelgrid_filter.setLeafSize(voxel_value_, voxel_value_, voxel_value_);
        voxelgrid_filter.filter(cloud);
        // ROS_INFO_STREAM("After voxel " << cloud.size());

        ///測定地点から離れすぎた点も除去するっぽいのでコメントアウト
        ///外れ値の除去
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud (cloud.makeShared());//外れ値を除去する点群を入力
        // sor.setMeanK (50);//MeanKを設定
        // sor.setStddevMulThresh (sor_threshold_);
        // sor.setNegative (false);//外れ値を出力する場合はtrueにする
        // sor.filter (cloud);//出力

        ///点群をカラーにremapし全体を白にする
        cloud_rgb.points.resize(cloud.size());
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            cloud_rgb.points[i].x = cloud.points[i].x;
            cloud_rgb.points[i].y = cloud.points[i].y;
            cloud_rgb.points[i].z = cloud.points[i].z;

            cloud_rgb.points[i].r = 255;
            cloud_rgb.points[i].g = 255;
            cloud_rgb.points[i].b = 255;
        }

        ///平面検出アルゴリズム
        // Create the segmentation object
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMaxIterations(500);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_threshold_);
    
        seg.setInputCloud(cloud_rgb.makeShared());
        seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));///認識する軸をz軸に限定
        seg.setEpsAngle(15.0f * (M_PI/180.0f));
        seg.segment(*inliers, *coefficients);

        ///検出した平面が大きく外れていた場合の例外処理
        if(coefficients->values[2] < 0){
            return;
        }

        ///平面を除去するかしないか.しない場合赤に染める
        if(plane_remove_){
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud_rgb.makeShared());
            extract.setIndices(inliers);
            extract.setNegative(true);// true にすると平面を除去、false にすると平面以外を除去
            extract.filter(cloud_rgb);
        }else{
            for (size_t i = 0; i < inliers->indices.size(); ++i)
            {
                cloud_rgb.points[inliers->indices[i]].r = 255;
                cloud_rgb.points[inliers->indices[i]].g = 0;
                cloud_rgb.points[inliers->indices[i]].b = 0;
            }
        }

        ///検出した平面を元に機体の姿勢を推定
        tf::StampedTransform transform = get_static_transform(m_lidar_link_frame_, m_base_link_frame_);
        float dist = std::abs(coefficients->values[0]*transform.getOrigin().x() + 
                                coefficients->values[1]*transform.getOrigin().y() +
                                coefficients->values[2]*transform.getOrigin().z() + coefficients->values[3]);
        tf::Vector3 dir(-coefficients->values[0]*dist, -coefficients->values[1]*dist, coefficients->values[2]*dist);
        // ROS_WARN_STREAM("Point: " << -coefficients->values[0]*dist << " , " << -coefficients->values[1]*dist << " , " << coefficients->values[2]*dist);
        tf::Transform br_transform;
        br_transform.setOrigin(dir);
        br_transform.setRotation(get_direction_quaternion(dir, tf::Vector3(0, 0, 1)));
        br_.sendTransform(tf::StampedTransform(br_transform, ros::Time::now(), footprint_frame_, base_link_frame_));

        pcl::toROSMsg(cloud_rgb, plane_pc2);

        ///点群の親フレームをfootprint_frameに設定(するように回転移動)
        br_transform *= get_static_transform(base_link_frame_, laser_frame_);
        pcl_ros::transformPointCloud(footprint_frame_, br_transform, plane_pc2, plane_pc2);
        plane_pc2.header.frame_id = footprint_frame_;
        plane_pc2.header.stamp = ros::Time::now();
        post_pointcloud_process(plane_pc2);
        plane_pub_.publish(plane_pc2);

        pcl_ros::transformPointCloud(footprint_frame_, br_transform, pc2, pc2);
        pc2.header.frame_id = footprint_frame_;
        pc2.header.stamp = ros::Time::now();
        pointcloud_pub_.publish(pc2);

        pcl_ros::transformPointCloud(footprint_frame_, br_transform, tmp_pc, tmp_pc);
        tmp_pc.header.frame_id = footprint_frame_;
        tmp_pc.header.stamp = ros::Time::now();
        pointcloud_tmp_pub_.publish(tmp_pc);
    }

    ///ポイントクラウドの事前処理
    ///現在位置に応じた事前マスク処理を実行
    void pre_pointcloud_process(sensor_msgs::PointCloud2 &pc2){
        ///現在地点の取得
        tf::StampedTransform transform;
        try{
            listener_.lookupTransform("/map", "/base_footprint",  ros::Time(0), transform);
        } catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            return;
        }

        tf::Transform org_tf, front_tf;
        org_tf.setIdentity();
        front_tf.setIdentity();

        ///ヨー軸回転のtransformを正回転と逆回転それぞれ取得
        tf::Quaternion org_qua = transform.getRotation();
        front_tf.setRotation(org_qua.inverse());
        org_tf.setRotation(org_qua);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (pc2, cloud);

        if(cloud.size() == 0){
            return;
        }

        /// マップのx軸正方向が正面となるようにポイントクラウドを回転
        pcl_ros::transformPointCloud<pcl::PointXYZ>(cloud, cloud, org_tf);

        switch (court_mode_)
        {
            case COURT_MODE_RED:
            case COURT_MODE_BLUE:
                {
                    // Create the filtering object
                    pcl::PassThrough<pcl::PointXYZ> pass;
                    pass.setInputCloud(cloud.makeShared());
                    pass.setFilterFieldName ("x");
                    pass.setFilterLimits (-2.0, field_pc_y_max_ - transform.getOrigin().x());
                    pass.filter (cloud);   

                    double x_min, x_max;
                    if(transform.getOrigin().x() > 5.0){ ///ゴール手前
                        // Create the filtering object
                        pcl::PassThrough<pcl::PointXYZ> pass;
                        pass.setInputCloud(cloud.makeShared());
                        pass.setFilterFieldName ("y");
                        
                        if(court_mode_ == COURT_MODE_RED){
                            x_max = -field_pc_x_min_ - transform.getOrigin().y();
                            x_min = -transform.getOrigin().y();
                        }else if(court_mode_ == COURT_MODE_BLUE){
                            x_max = -transform.getOrigin().y();
                            x_min = -(transform.getOrigin().y() - field_pc_x_min_);
                        }else{
                            ROS_ERROR_STREAM("court mode error 2");
                            return;
                        }

                        pass.setFilterLimits (x_min, x_max);
                        pass.filter (cloud);
                    }else{ 
                        // Create the filtering object
                        pcl::PassThrough<pcl::PointXYZ> pass;
                        pass.setInputCloud(cloud.makeShared());
                        pass.setFilterFieldName ("y");

                        if(court_mode_ == COURT_MODE_RED){
                            x_max = -field_pc_x_min_ - transform.getOrigin().y();
                            x_min = -(transform.getOrigin().y() + field_pc_x_max_);
                        }else if(court_mode_ == COURT_MODE_BLUE){
                            x_max = field_pc_x_max_ - transform.getOrigin().y();
                            x_min = -(transform.getOrigin().y() - field_pc_x_min_);
                        }else{
                            ROS_ERROR_STREAM("court mode error 3");
                            return;
                        }

                        pass.setFilterLimits (x_min, x_max);
                        pass.filter (cloud);    
                    }
                break;
                }

            case COURT_MODE_RED_MT:
            case COURT_MODE_BLUE_MT:

                break;

            default:
                ROS_WARN_STREAM("court mode error 1");
                break;
        }


        ///回転したポイントクラウドを元に戻す
        pcl_ros::transformPointCloud<pcl::PointXYZ>(cloud, cloud, front_tf);
        pcl::toROSMsg(cloud, pc2);
    }

    ///ポイントクラウドの事後処理
    ///ｚ軸から特定の範囲を抽出し，地面に投影する．
    void post_pointcloud_process(sensor_msgs::PointCloud2 &pc2){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (pc2, cloud);

        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_z_min_, filter_z_max_);
        //pass.setFilterLimitsNegative (true);
        pass.filter (cloud);

        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            cloud.points[i].z = 0;
        }

        pcl::toROSMsg(cloud, pc2);
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
        ROS_INFO_STREAM("Call court mode2(plane estimation) " << mode);
        court_mode_ = (court_mode_t)req.court_mode.data;
        res.success = true;
        return true;
    }
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_estimation_node");

    plane_estimation pe;

    ros::spin();
}
