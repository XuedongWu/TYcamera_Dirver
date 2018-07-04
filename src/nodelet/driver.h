//
// Created by weeksun on 18-3-14.
//

#ifndef CAMPORT_DRIVER_H
#define CAMPORT_DRIVER_H


#include "common.hpp"


#include <nodelet/nodelet.h>
#include <functional>
#include <memory>

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <unistd.h>


// ROS communication
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>

// Configuration
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/distortion_models.h>

// diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace camport{

    class DriverNodelet:public nodelet::Nodelet{
    public:
        DriverNodelet();
        virtual ~DriverNodelet ();

    private:
        struct CallbackData {
            int             index;
            TY_DEV_HANDLE   hDevice;
            DepthRender*    render;

            TY_CAMERA_DISTORTION color_dist;
            TY_CAMERA_INTRINSIC color_intri;
        };

        /** \brief Nodelet initialization routine. */
        virtual void onInit ();
        /**
         * @brief 初始化相机
         */
        void onInitCamera();
        /**
         * @brief 启动相机传感器驱动，初始化对应流对象
         * @param param_nh 节点操作句柄
         * @return 启动成功返回true,失败返回false
         */
        bool startInuSensor(ros::NodeHandle& param_nh);
        /**
         * @brief 获取相机光学参数
         */
        void getOpticalParam();
        /**
         * @brief 初始化深度图流对象
         * @return 初始化成功返回ture,失败返回false
         */
        bool InitDepth();
        bool InitRgb();
        bool InitIr();
        bool InitDepthReg();

        void depth2Point(const cv::Mat& depth,cv::Mat& p3d);

        bool developer_mode = false;
        bool has_color_ = false;
        bool has_depth_ = false;
        bool has_point3d_ = false;
        bool has_depth_intrinsic_ = false;
        bool old_device_          = false;

        void frameHandler(TY_FRAME_DATA* frame, void* userdata);
        void publishRgbImage(const cv::Mat&  iFrame, ros::Time time) const;
        void publishDepthImage(const cv::Mat&  iFrame, ros::Time time,bool reg) const;
        void publishPoint3d(const cv::Mat& iFrame,ros::Time time) const;
        sensor_msgs::CameraInfoPtr getRGBCameraInfo(int width, int height,ros::Time stamp) const;
        sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height,ros::Time stamp) const;
        //成员变量
        //相机初始化线程类
        boost::thread init_thread_;
        bool startDepth;
        bool startRgb;
        bool startIr;
        bool startDepthReg;
        bool initedDepth;
        bool initedRgb;
        bool initedIr;
        bool initedDepthReg;

        bool connectedDepth;
        bool connectedRgb;
        bool connectedIr;
        bool connectedDepthReg;


        char buffer_[1024*1024];
        int  n_;
        //相机结构参数
        TY_CAMERA_INTRINSIC rgb_camera_intr_;    ///< rgb相机内参
        TY_CAMERA_INTRINSIC depth_camera_intr_;  ///< depth相机内参
        TY_CAMERA_INTRINSIC default_rgb_camera_intr_;
        TY_CAMERA_INTRINSIC default_depth_camera_intr_;


        // Counters/flags for skipping frames
        boost::mutex counter_mutex_;
        int rgb_frame_counter_;
        int depth_frame_counter_;
        int ir_frame_counter_;
        bool publish_rgb_;
        bool publish_ir_;
        bool publish_depth_;


        // published topics
        image_transport::CameraPublisher pub_rgb_;
        image_transport::CameraPublisher pub_depth_, pub_depth_registered_;
        ros::Publisher pub_point3d_;



        /** \brief Camera info manager objects. */
        boost::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, ir_info_manager_;
        std::string rgb_frame_id_;
        std::string depth_frame_id_;
    };
}

#endif //CAMPORT_DRIVER_H
