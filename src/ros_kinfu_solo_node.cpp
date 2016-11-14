#include <iostream>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

namespace ns_kinfuls = pcl::gpu::kinfuLS;
namespace ns_dev_kinfuls = pcl::device::kinfuLS;

class KinfuLSApp
{
private:
    ros::NodeHandle _nh;
    float _width, _height;
    float _volume_size, _shift_distance;
    int _frame_counter, _snapshot_rate;
    bool _is_lost;
    boost::shared_ptr<ns_kinfuls::KinfuTracker> _kinfu;
    ns_kinfuls::KinfuTracker::DepthMap _depth_device;
    ns_kinfuls::KinfuTracker::View _color_device;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _tsdf_cloud_ptr;
    sensor_msgs::ImageConstPtr _depth_ptr;
    sensor_msgs::ImageConstPtr _color_ptr;

    ros::Subscriber _sub_depth;
    ros::Subscriber _sub_color;
    std::string _topic_view; // empty if disable

    void execute()
    {
        bool has_image = false;
        ++_frame_counter;

        _is_lost = _kinfu->icpIsLost();

        if(_depth_ptr)
        {
            _depth_device.upload((const void*)&_depth_ptr->data.front(), _depth_ptr->step,
                               _depth_ptr->height, _depth_ptr->width);
            if(_color_ptr)
            {
                _color_device.upload((const void*)&_color_ptr->data.front(), _color_ptr->step,
                                     _color_ptr->height, _color_ptr->width);
                has_image = (*_kinfu)(_depth_device, _color_device);
            }
            else
            {
                has_image = (*_kinfu)(_depth_device);
            }

            publishView(_topic_view);
        }
        _depth_ptr.reset();
        _color_ptr.reset();
    }

    void loop()
    {
        ros::Rate rate(30.0);
        while(!ros::isShuttingDown())
        {
            execute();
            rate.sleep();
        }
    }

    void cbDepth(const sensor_msgs::ImageConstPtr &msg)
    {
        _depth_ptr = msg;
    }

    void cbColor(const sensor_msgs::ImageConstPtr &msg)
    {
        _color_ptr = msg;
    }

    void loadParams()
    {
        typedef sensor_msgs::CameraInfo INFO;
        // directly available parameters
        _height = (float)_nh.param<double>("height", -1.0);
        _width = (float)_nh.param<double>("width", -1.0);
        // topics
        std::string topic_depth, topic_color, topic_info;
        topic_depth = _nh.param<std::string>("topic_depth", "");
        topic_color = _nh.param<std::string>("topic_image", "");
        topic_info = _nh.param<std::string>("topic_camera_info", "");
        INFO::ConstPtr info = ros::topic::waitForMessage<INFO>(topic_info, _nh, ros::Duration(2));
        _topic_view = _nh.param<std::string>("topic_view", "");
        if(topic_depth.empty() || topic_color.empty() || topic_info.empty()
                || _height < 0.0f || _width < 0.0f || !info)
        {
            ROS_ERROR("parameter error");
            ros::shutdown();
        }
        // should be safe, apply
        _sub_depth = _nh.subscribe<sensor_msgs::Image>(topic_depth, 1, &KinfuLSApp::cbDepth, this);
        _sub_color = _nh.subscribe<sensor_msgs::Image>(topic_color, 1, &KinfuLSApp::cbColor, this);
        _kinfu->setDepthIntrinsics(info->K.at(0), info->K.at(4), info->K.at(2), info->K.at(5));
    }

    void publishView(const std::string topic)
    {
        if(topic.empty())
        {
            return;
        }
        static ros::Publisher pub = _nh.advertise<sensor_msgs::Image>(topic, 1, true);
        // publish image, will be moved to another place
        ns_kinfuls::KinfuTracker::View view_device;
        std::vector<ns_kinfuls::PixelRGB> view_host;
        int cols;
        sensor_msgs::ImagePtr msg = sensor_msgs::ImagePtr(new sensor_msgs::Image());
        _kinfu->getImage(view_device);
        view_device.download(view_host, cols);
        sensor_msgs::fillImage(*msg, "rgb8", view_device.rows(), view_device.cols(),
                               view_device.cols() * 3, &view_host.front());
        pub.publish(msg);
    }

public:
    KinfuLSApp(const float volume_size, const float shift_distance, const int snapshot_rate,
               const float width, const float height):
        _nh(ros::NodeHandle("~")), // may replace to private etc
        _volume_size(volume_size),
        _shift_distance(shift_distance),
        _snapshot_rate(snapshot_rate),
        _is_lost(false),
        _kinfu(new ns_kinfuls::KinfuTracker(Eigen::Vector3f::Constant(_volume_size),
                                            _shift_distance)),
        _topic_view("")
    {
        // modified from or directly from kinfuLS_app.cpp, pcl 1.8
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
        Eigen::Vector3f t = Eigen::Vector3f::Constant(_volume_size) * 0.5f
                - Eigen::Vector3f (0, 0, _volume_size / 2 * 1.2f);

        Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

        _kinfu->setInitialCameraPose(pose);
        _kinfu->volume().setTsdfTruncDist (0.030f/*meters*/);
        _kinfu->setIcpCorespFilteringParams (0.1f/*meters*/, sin (M_PI * 20.0 / 180.0));
        //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
        _kinfu->setCameraMovementThreshold(0.001f);

        //Init KinFuLSApp
        _tsdf_cloud_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        _frame_counter = 0;

        loadParams();

        boost::thread worker(boost::bind(&KinfuLSApp::loop, this));
    }
    ~KinfuLSApp()
    {
    }
};

int main(int argn, char **argv)
{
    ros::init(argn, argv, "ros_kinfu_solo");

    KinfuLSApp app(3.0f, 1.8f, ns_dev_kinfuls::SNAPSHOT_RATE, 640.0f, 480.0f);

    ros::spin();
    return 0;
}
