//
// Created by weeksun on 17-4-25.
//

#include "driver.h"

using namespace camport;


DriverNodelet::DriverNodelet(){

}

DriverNodelet::~DriverNodelet (){

}


void DriverNodelet::onInit (){
    init_thread_ = boost::thread(boost::bind(&DriverNodelet::onInitCamera, this));
}

void DriverNodelet::onInitCamera(){
    ros::NodeHandle &nh = getNodeHandle();        // topics
    ros::NodeHandle &param_nh = getPrivateNodeHandle(); // parameters

    // Allow remapping namespaces rgb, ir, depth, depth_registered
    image_transport::ImageTransport it(nh);
    ros::NodeHandle rgb_nh(nh, "rgb");
    image_transport::ImageTransport rgb_it(rgb_nh);
    ros::NodeHandle ir_nh(nh, "ir");
    image_transport::ImageTransport ir_it(ir_nh);
    ros::NodeHandle depth_nh(nh, "depth");
    image_transport::ImageTransport depth_it(depth_nh);
    ros::NodeHandle depth_registered_nh(nh, "depth_registered");
    image_transport::ImageTransport depth_registered_it(depth_registered_nh);


    rgb_frame_counter_ = depth_frame_counter_ = ir_frame_counter_ = 0;
    publish_rgb_ = publish_ir_ = publish_depth_ = true;

    // Camera TF frames
    param_nh.param("rgb_frame_id", rgb_frame_id_, std::string("/camport_rgb_optical_frame"));
    param_nh.param("depth_frame_id", depth_frame_id_, std::string("/camport_depth_optical_frame"));
    NODELET_INFO("rgb_frame_id = '%s' ", rgb_frame_id_.c_str());
    NODELET_INFO("depth_frame_id = '%s' ", depth_frame_id_.c_str());


    TY_DEV_HANDLE hDevice;
    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer_;
    ASSERT_OK( TYLibVersion(pVer) );
    LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    LOGD("=== Get device info");
    ASSERT_OK( TYGetDeviceNumber(&n_) );
    LOGD("     - device number %d", n_);

    TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer_;


    while(ros::ok()){
      ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n_) );

      if(n_ == 0){
        LOGD("=== No device got");
        sleep(1);
      }else{
        break;
      }
    }


    LOGD("=== Open device 0");
    ASSERT_OK( TYOpenDevice(pBaseInfo[0].id, &hDevice) );

    if(developer_mode){
      LOGD("=== Enter Developer Mode");
      ASSERT_OK(TYEnterDeveloperMode(hDevice));
    }



    int32_t allComps=0;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );

    if(allComps & TY_COMPONENT_POINT3D_CAM){
      LOGD("=== Configure components, open point3d cam");
      // int32_t componentIDs = TY_COMPONENT_POINT3D_CAM;
      int32_t componentIDs = TY_COMPONENT_POINT3D_CAM;
      ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );
      has_point3d_ = true;
    }


    if(allComps & TY_COMPONENT_RGB_CAM){


        int err = TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM,
                              TY_STRUCT_CAM_INTRINSIC, (void*)&rgb_camera_intr_, sizeof(rgb_camera_intr_));
        if(err != TY_STATUS_OK){
            LOGE("Get camera RGB intrinsic failed: %s", TYErrorString(err));
        } else {
            has_color_ = true;
            LOGD("=== Has RGB camera, open RGB cam");
            ASSERT_OK( TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM) );
        }

        LOGD("=== Configure feature, set rgb resolution to 640x480.");
        TY_STATUS errs = TYSetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
        ASSERT(errs == TY_STATUS_OK || errs == TY_STATUS_NOT_PERMITTED);
    }

    if(allComps & TY_COMPONENT_DEPTH_CAM){


      int err = TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM,
                            TY_STRUCT_CAM_INTRINSIC,
                            (void*)&depth_camera_intr_, sizeof(depth_camera_intr_));
      if(err != TY_STATUS_OK){
          LOGE("Get camera depth intrinsic failed: %s", TYErrorString(err));
      } else {
          has_depth_ = true;
          LOGD("=== Has depth camera, open depth cam");
          ASSERT_OK( TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM) );
      }


    }

    LOGD("=== Configure feature, set depth resolution to 640x480.");
    LOGD("Note: DM460 resolution feature is in component TY_COMPONENT_DEVICE,");
    LOGD("      other device may lays in some other components.");
    TY_STATUS err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
    ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

    LOGD("=== Prepare image buffer");
    int32_t frameSize;
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);
    ASSERT( frameSize >= 640*480*2 );

    LOGD("     - Allocate & enqueue buffers");
    char* frameBuffer[2];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );

    LOGD("=== Register callback");
    LOGD("Note: Callback may block internal data receiving,");
    LOGD("      so that user should not do long time work in callback.");
    LOGD("      To avoid copying data, we pop the framebuffer from buffer queue and");
    LOGD("      give it back to user, user should call TYEnqueueBuffer to re-enqueue it.");
    DepthRender render;
    CallbackData cb_data;
    cb_data.index = 0;
    cb_data.hDevice = hDevice;
    cb_data.render = &render;
    // ASSERT_OK( TYRegisterCallback(hDevice, frameHandler, &cb_data) );

    LOGD("Publish camera topics");
    pub_depth_ = depth_it.advertiseCamera("image_raw",1);
    pub_rgb_ = rgb_it.advertiseCamera("image_raw",1);
    pub_point3d_ = depth_nh.advertise<sensor_msgs::PointCloud2>(
          "point3d",1);

    LOGD("=== Disable trigger mode");
    ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false) );

    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

    LOGD("=== While loop to fetch frame");

    TY_FRAME_DATA frame;

    while(ros::ok()){
        int err = TYFetchFrame(hDevice, &frame, -1);
        if( err != TY_STATUS_OK ){
            LOGD("... Drop one frame");
            continue;
        }

        frameHandler(&frame, &cb_data);

        if(developer_mode){
          DEVELOPER_MODE_PRINT();
        }

    }

    ASSERT_OK( TYStopCapture(hDevice) );
    ASSERT_OK( TYCloseDevice(hDevice) );
    ASSERT_OK( TYDeinitLib() );
    // MSLEEP(10); // sleep to ensure buffer is not used any more
    delete frameBuffer[0];
    delete frameBuffer[1];

    LOGD("=== Main done!");
    return ;
}

void DriverNodelet::frameHandler(TY_FRAME_DATA* frame, void* userdata){
  CallbackData* pData = (CallbackData*) userdata;
  //LOGD("=== Get frame %d", ++pData->index);

  cv::Mat depth, color, p3d;
  parseFrame(*frame, &depth, 0, 0, &color, &p3d);
  ros::Time stamp = ros::Time::now();

  if(!depth.empty()){

    publishDepthImage(depth,stamp,false);
  }

  if(!color.empty()){
    publishRgbImage(color,stamp);
    //cv::imshow("few",color);
  }

  if(!p3d.empty()){
    publishPoint3d(p3d,stamp);
  }

//  cv::waitKey(1);

  //LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
  ASSERT_OK( TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize) );
}


void DriverNodelet::publishRgbImage(const cv::Mat& iFrame, ros::Time time) const {

  std_msgs::Header header;
  header.frame_id = rgb_frame_id_;
  header.stamp = time;
  cv_bridge::CvImage cv_image(header,sensor_msgs::image_encodings::BGR8,iFrame);

  pub_rgb_.publish(cv_image.toImageMsg(),
                   getRGBCameraInfo(iFrame.cols, iFrame.rows,time));
}

void DriverNodelet::publishDepthImage(const cv::Mat &iFrame, ros::Time time, bool reg) const{

    std_msgs::Header header;
    header.frame_id = depth_frame_id_;
    header.stamp = time;
    cv_bridge::CvImage cv_image(header,sensor_msgs::image_encodings::MONO16,iFrame);

    if(reg){
//        pub_depth_registered_.publish(depth_msg.toImageMsg(),
//                                      getRGBCameraInfo(iFrame->getWidth(),
//                                                       iFrame->getHeight(),time));
    }else{
        pub_depth_.publish(cv_image.toImageMsg(),
                           getDepthCameraInfo(iFrame.cols,
                                            iFrame.rows,time));
    }
}

void DriverNodelet::publishPoint3d(const cv::Mat& iFrame,ros::Time time) const{
  pcl::PointCloud<pcl::PointXYZ> pc;
  pc.header.frame_id = depth_frame_id_;
  pc.header.stamp = pcl_conversions::toPCL(time);
  pc.width = iFrame.cols;
  pc.height = iFrame.rows;

  for(int i=0;i<iFrame.rows;i++){
    for(int j=0;j<iFrame.cols;j++){
      cv::Vec3f p = iFrame.at<cv::Vec3f>(i,j);
      p /= 1000.0;
      pc.points.push_back(pcl::PointXYZ(p[0],p[1],p[2]));
    }
  }


  sensor_msgs::PointCloud2 p3d_msg;

  pcl::toROSMsg(pc,p3d_msg);

  pub_point3d_.publish(p3d_msg);

}

sensor_msgs::CameraInfoPtr DriverNodelet::getRGBCameraInfo(int width, int height,ros::Time stamp) const {
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->header.frame_id = rgb_frame_id_;
    info->header.stamp = stamp;
    //info->header.seq = rgb_seq;
    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D[0] = 0;
    info->D[1] = 0;
    info->D[2] = 0;
    info->D[3] = 0;
    info->D[4] = 0;
    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = rgb_camera_intr_.data[0]; //fx
    info->K[4] = rgb_camera_intr_.data[4]; // fy
    info->K[2] = rgb_camera_intr_.data[2];     // cx
    info->K[5] = rgb_camera_intr_.data[5];     // cy
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0]  = rgb_camera_intr_.data[0]; //fx
    info->P[5] = rgb_camera_intr_.data[4]; // fy
    info->P[2]  = rgb_camera_intr_.data[2];     // cx
    info->P[6]  = rgb_camera_intr_.data[5];     // cy
    info->P[10] = 1.0;

    return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDepthCameraInfo(int width, int height,ros::Time stamp) const{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  info->header.frame_id = depth_frame_id_;
  info->header.stamp = stamp;
  //info->header.seq = rgb_seq;
  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->D[0] = 0;
  info->D[1] = 0;
  info->D[2] = 0;
  info->D[3] = 0;
  info->D[4] = 0;
  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = depth_camera_intr_.data[0]; //fx
  info->K[4] = depth_camera_intr_.data[4]; // fy
  info->K[2] = depth_camera_intr_.data[2];     // cx
  info->K[5] = depth_camera_intr_.data[5];     // cy
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = depth_camera_intr_.data[0]; //fx
  info->P[5] = depth_camera_intr_.data[4]; // fy
  info->P[2]  = depth_camera_intr_.data[2];     // cx
  info->P[6]  = depth_camera_intr_.data[5];     // cy
  info->P[10] = 1.0;

  return info;
}





// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (camport, driver, camport::DriverNodelet, nodelet::Nodelet);
