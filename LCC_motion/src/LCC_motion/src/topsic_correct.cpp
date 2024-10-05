#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// 定义 Ouster 点云结构体
namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t  ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// 注册 Ouster 点云结构体
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// 定义 Velodyne 点云结构体
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// 注册 Velodyne 点云结构体
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<CompressedImage, CameraInfo, PointCloud2> MySyncPolicy;

class DataSynchronizer {
public:
    DataSynchronizer() {
        image_sub.subscribe(nh, "/stereo/vehicle_frame_left/image_raw/compressed", 1);
        camera_info_sub.subscribe(nh, "/stereo/vehicle_frame_left/camera_info", 1);
        point_cloud_sub.subscribe(nh, "/os_cloud_node/points", 1);

        sync.reset(new Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, camera_info_sub, point_cloud_sub));
        sync->registerCallback(boost::bind(&DataSynchronizer::callback, this, _1, _2, _3));

        pub_image = nh.advertise<Image>("/stereo/vehicle_frame_left/image_raw", 1);
        pub_camera_info = nh.advertise<CameraInfo>("/stereo/vehicle_frame_left_image_raw/camera_info", 1);
        pub_point_cloud = nh.advertise<PointCloud2>("/osve_cloud_node/points", 1);
    }

    void callback(const CompressedImageConstPtr& image_msg,
                  const CameraInfoConstPtr& camera_info_msg,
                  const PointCloud2ConstPtr& point_cloud_msg) {
        
        // 解压图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 修改图像（如有需要）
        cv::Mat& image = cv_ptr->image;

        // 从压缩图像生成新的Image消息
        sensor_msgs::ImagePtr new_image_msg = cv_ptr->toImageMsg();
        new_image_msg->header = image_msg->header;

        // 修改点云格式
        pcl::PointCloud<ouster_ros::Point> ouster_cloud;
        pcl::fromROSMsg(*point_cloud_msg, ouster_cloud);

        pcl::PointCloud<velodyne_ros::Point> velodyne_cloud;
        for (const auto& point : ouster_cloud.points) {
            velodyne_ros::Point velodyne_point;
            velodyne_point.x = point.x;
            velodyne_point.y = point.y;
            velodyne_point.z = point.z;
            velodyne_point.intensity = point.intensity;
            velodyne_point.time = static_cast<float>(point.t) / 1e9; // 将时间从纳秒转换为秒
            velodyne_point.ring = point.ring;
            velodyne_cloud.points.push_back(velodyne_point);
        }
        velodyne_cloud.header = ouster_cloud.header;

        // 将点云转换为ROS消息
        sensor_msgs::PointCloud2 modified_point_cloud_msg;
        pcl::toROSMsg(velodyne_cloud, modified_point_cloud_msg);

        // 发布修改后的消息
        pub_image.publish(new_image_msg);
        pub_camera_info.publish(camera_info_msg);
        pub_point_cloud.publish(modified_point_cloud_msg);
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<CompressedImage> image_sub;
    message_filters::Subscriber<CameraInfo> camera_info_sub;
    message_filters::Subscriber<PointCloud2> point_cloud_sub;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher pub_image;
    ros::Publisher pub_camera_info;
    ros::Publisher pub_point_cloud;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_synchronizer");
    DataSynchronizer ds;
    ros::spin();
    return 0;
}
