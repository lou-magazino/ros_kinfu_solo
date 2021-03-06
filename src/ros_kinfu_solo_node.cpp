#include <iostream>
#include <vector>
#include <string>
#include <csignal>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <ros/xmlrpc_manager.h>

// from local source code
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>

namespace ns_kinfuls = pcl::gpu::kinfuLS;
namespace ns_dev_kinfuls = pcl::device::kinfuLS;

sig_atomic_t volatile g_request_shutdown = 0; // for saving mesh at the end of program

class KinfuLSApp
{
private:
    ros::NodeHandle _nh;
    int _width, _height;
    float _volume_size, _shift_distance;
    int _frame_counter, _snapshot_rate;
    bool _is_lost;
    boost::shared_ptr<ns_kinfuls::KinfuTracker> _kinfu;
    ns_kinfuls::KinfuTracker::DepthMap _depth_device;
    ns_kinfuls::KinfuTracker::View _color_device;
    sensor_msgs::ImageConstPtr _depth_ptr;
    sensor_msgs::ImageConstPtr _color_ptr;

    ros::Subscriber _sub_depth;
    ros::Subscriber _sub_color;
    std::string _topic_view; // empty if disable
    boost::thread _thread_kinfu;
    boost::thread _thread_save_mesh;

    /**
     * @brief execute real part of processing
     */
    void execute()
    {
        if(_depth_ptr)
        {
            // save local pointers to make sure they are "sync"
            sensor_msgs::ImageConstPtr depth_ptr(_depth_ptr);
            sensor_msgs::ImageConstPtr color_ptr(_color_ptr);
            _depth_ptr.reset();
            _color_ptr.reset();

            _is_lost = _kinfu->icpIsLost();
            ++_frame_counter;
            // upload depth map
            _depth_device.upload((const void*)&depth_ptr->data.front(), depth_ptr->step,
                                 depth_ptr->height, depth_ptr->width);
            if(color_ptr)
            {
                // if rgb data available, upload and proceed with it
                _color_device.upload((const void*)&color_ptr->data.front(), color_ptr->step,
                                     color_ptr->height, color_ptr->width);
                (*_kinfu)(_depth_device, _color_device);
            }
            else
            {
                // not rgb data, just depth
                (*_kinfu)(_depth_device);
            }

            // publish current view, will skip if _topic_view is empty
            publishView(_topic_view);
//            saveToMesh();
        } // skip if no data
    }

    /**
     * @brief executeLoop contains infinite loop for execute
     */
    void executeLoop()
    {
        ros::Rate rate(30.0); // assume 30hz, common for openni device
        while(ros::ok() && !g_request_shutdown)
        {
            execute();
            rate.sleep();
            boost::this_thread::interruption_point();
        }
    }

    /**
     * @brief cbDepth saves pointer to depth map from device's publisher
     * @param msg
     */
    void cbDepth(const sensor_msgs::ImageConstPtr &msg)
    {
        _depth_ptr = msg;
    }

    /**
     * @brief cbColor saves pointer to image from device's publisher
     * @param msg
     */
    void cbColor(const sensor_msgs::ImageConstPtr &msg)
    {
        _color_ptr = msg;
    }

    /**
     * @brief init reads params from launch file or others, then start
     *            subscribers and set parameters accordingly
     */
    void init()
    {
        typedef sensor_msgs::CameraInfo INFO;
        // directly available parameters
        _height = _nh.param<int>("height", -1);
        _width = _nh.param<int>("width", -1);
        // topics
        std::string topic_depth, topic_color, topic_info;
        topic_depth = _nh.param<std::string>("topic_depth", "");
        topic_color = _nh.param<std::string>("topic_image", "");
        topic_info = _nh.param<std::string>("topic_camera_info", "");
        INFO::ConstPtr info = ros::topic::waitForMessage<INFO>(topic_info, _nh, ros::Duration(2));
        _topic_view = _nh.param<std::string>("topic_view", "");
        if(topic_depth.empty() || topic_color.empty() || topic_info.empty()
                || _height < 0 || _width < 0 || !info)
        {
            ROS_ERROR("parameter error");
            ros::shutdown();
        }
        
        // init and configure kinfu here
        // modified from or directly from kinfuLS_app.cpp, pcl 1.8
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
        Eigen::Vector3f t = Eigen::Vector3f::Constant(_volume_size) * 0.5f
                - Eigen::Vector3f (0, 0, _volume_size / 2 * 1.2f);

        Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
        _kinfu = boost::make_shared<ns_kinfuls::KinfuTracker>(
	    ns_kinfuls::KinfuTracker(Eigen::Vector3f::Constant(_volume_size), _shift_distance, _height, _width));

        _kinfu->setInitialCameraPose(pose);
        _kinfu->volume().setTsdfTruncDist(0.030f/*meters*/);
        _kinfu->setIcpCorespFilteringParams(0.1f/*meters*/, sin(M_PI * 20.0 / 180.0));
        //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
        _kinfu->setCameraMovementThreshold(0.001f);
	
        // should be safe, apply
        _sub_depth = _nh.subscribe<sensor_msgs::Image>(topic_depth, 10, &KinfuLSApp::cbDepth, this);
//        _sub_color = _nh.subscribe<sensor_msgs::Image>(topic_color, 10, &KinfuLSApp::cbColor, this);
        _kinfu->setDepthIntrinsics(info->K.at(0), info->K.at(4), info->K.at(2), info->K.at(5));
    }

    /**
     * @brief publishView publishes current view to topic, skip if topic name is empty
     * @param topic
     */
    void publishView(const std::string topic)
    {
        if(!topic.empty())
        {
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
    }

    /**
     * @brief saveToMesh downloads tsdf, store to mesh(this part is currently part of pcl, may rewrite)
     */
    void saveToMesh()
    {
//        _kinfu->extractAndSaveWorld();

        // todo for this part:
        // pcl::PointCloud<pcl::PointXYZI>::Ptr KinfuTracker::getWorld();
        // generally same as extractAndSaveWorld, but returns cyclical_.getWorldModel ()->getWorld ()
        ROS_WARN("saving");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world = _kinfu->getWorld();
        if(!cloud_world->empty())
        {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > transforms;
            pcl::kinfuLS::WorldModel<pcl::PointXYZI> world;
            //Adding current cloud to the world model
            world.addSlice(cloud_world);
            //Get world as a vector of cubes
            world.getWorldAsCubes (ns_dev_kinfuls::VOLUME_X, clouds, transforms, 0.025); // 2.5% overlapp (12 cells with a 512-wide cube)

            pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> cubes(ns_dev_kinfuls::VOLUME_X,
                                                                             ns_dev_kinfuls::VOLUME_Y,
                                                                             ns_dev_kinfuls::VOLUME_Z,
                                                                             _volume_size);
            cubes.getMeshesFromTSDFVector(clouds, transforms); // ply file saved to ./ros/mesh*.ply, return to somewhere
        }
        else
        {
            ROS_ERROR("no cloud available");
        }
    }

    void saveToMeshLoop()
    {
        ros::Rate r(1.0); // per second
        while(ros::ok() && !g_request_shutdown)
        {
            r.sleep();
            boost::this_thread::interruption_point();
        }
        saveToMesh(); // save when not shutdown, if ros is not okay, this would not be called
    }

public:
    KinfuLSApp(const float volume_size, const float shift_distance, const int snapshot_rate):
        _nh(ros::NodeHandle("~")), // may replace to private etc
        _volume_size(volume_size),
        _shift_distance(shift_distance),
        _snapshot_rate(snapshot_rate),
        _is_lost(false),
        _topic_view("")
    {
        _frame_counter = 0;
        // read parameters
        init();
        // start thread for process
        startThread();
    }

    ~KinfuLSApp()
    {
    }

    /**
     * @brief stopThread
     */
    void stopThread()
    {
        // interrupt and joinm interrupt point is after execution step, so should be okay
        _thread_kinfu.interrupt();
        _thread_kinfu.join();
        _thread_save_mesh.interrupt();
        _thread_save_mesh.join();
    }

    /**
     * @brief startThread
     */
    void startThread()
    {
        // restart by making a new thread
        _thread_kinfu = boost::thread(boost::bind(&KinfuLSApp::executeLoop, this));
        _thread_save_mesh = boost::thread(boost::bind(&KinfuLSApp::saveToMeshLoop, this));
    }
};

void sig_int_handler_shutdown(int sig)
{
    ROS_INFO("Shutdown request");
    g_request_shutdown = 1;
}

// http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
void callback_shutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    ROS_INFO("Shutdown request");
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        num_params = params.size();
    }
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argn, char **argv)
{
    ros::init(argn, argv, "ros_kinfu_solo", ros::init_options::NoSigintHandler);

    KinfuLSApp app(3.0f, 1.5f, ns_dev_kinfuls::SNAPSHOT_RATE);

    signal(SIGINT, sig_int_handler_shutdown);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", callback_shutdown);

    ros::spin();
    return 0;
}
