#include "../common/common.hpp"

static char buffer[1024*1024];
static int  n;
static volatile bool exit_main;


struct CallbackData {
    int             index;
    TY_DEV_HANDLE   hDevice;
    DepthRender*    render;
    PointCloudViewer* pcviewer;
};

void frameHandler(TY_FRAME_DATA* frame, void* userdata)
{
    CallbackData* pData = (CallbackData*) userdata;
    LOGD("=== Get frame %d", ++pData->index);

    cv::Mat depth;
    parseFrame(*frame, &depth, 0, 0, 0, 0);

    if(!depth.empty()){
        cv::Mat colorDepth = pData->render->Compute(depth);
        cv::imshow("ColorDepth", colorDepth);

        // conver depth to world
        static TY_VECT_3F depthbuf[1280*960];
        static TY_VECT_3F worldbuf[1280*960];
        int k = 0;
        uint16_t* pdepth = (uint16_t*)depth.data;
        for(int r = 0; r < depth.rows; r++)
            for(int c = 0; c < depth.cols; c++){
                depthbuf[k].x = c;
                depthbuf[k].y = r;
                depthbuf[k].z = pdepth[k];
                k++;
            }
        ASSERT_OK( TYDepthToWorld(pData->hDevice, depthbuf, worldbuf, 0, depth.rows*depth.cols) );

        // show point3d
        cv::Mat point3D(depth.rows, depth.cols, CV_32FC3, (void*)worldbuf);
        pData->pcviewer->show(point3D, "depth2world");
        if(pData->pcviewer->isStopped("depth2world")){
            exit_main = true;
            return;
        }
    }

    int key = cv::waitKey(1);
    switch(key){
        case -1:
            break;
        case 'q': case 1048576 + 'q':
            exit_main = true;
            break;
        default:
            LOGD("Pressed key %d", key);
    }

    LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
    ASSERT_OK( TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize) );
}

int main()
{
    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer;
    ASSERT_OK( TYLibVersion(pVer) );
    LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    LOGD("=== Get device info");
    ASSERT_OK( TYGetDeviceNumber(&n) );
    LOGD("     - device number %d", n);

    TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
    ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n) );

    if(n == 0){
        LOGD("=== No device got");
        return -1;
    }

    LOGD("=== Open device 0");
    TY_DEV_HANDLE hDevice;
    ASSERT_OK( TYOpenDevice(pBaseInfo[0].id, &hDevice) );

#ifdef DEVELOPER_MODE
    LOGD("=== Enter Developer Mode");
    ASSERT_OK(TYEnterDeveloperMode(hDevice));
#endif

    LOGD("=== Configure components, open depth cam");
    int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );

    LOGD("=== Configure feature, set resolution to 640x480.");
    LOGD("Note: DM460 resolution feature is in component TY_COMPONENT_DEVICE,");
    LOGD("      other device may lays in some other components.");
    int err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
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
    PointCloudViewer pcviewer;
    CallbackData cb_data;
    cb_data.index = 0;
    cb_data.hDevice = hDevice;
    cb_data.render = &render;
    cb_data.pcviewer = &pcviewer;
    // ASSERT_OK( TYRegisterCallback(hDevice, frameHandler, &cb_data) );

    LOGD("=== Disable trigger mode");
    ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false) );

    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

    LOGD("=== While loop to fetch frame");
    exit_main = false;
    TY_FRAME_DATA frame;

    while(!exit_main){
        int err = TYFetchFrame(hDevice, &frame, -1);
        if( err != TY_STATUS_OK ){
            LOGD("... Drop one frame");
            continue;
        }

        frameHandler(&frame, &cb_data);

#ifdef DEVELOPER_MODE
        DEVELOPER_MODE_PRINT();
#endif
    }

    ASSERT_OK( TYStopCapture(hDevice) );
    ASSERT_OK( TYCloseDevice(hDevice) );
    ASSERT_OK( TYDeinitLib() );
    // MSLEEP(10); // sleep to ensure buffer is not used any more
    delete frameBuffer[0];
    delete frameBuffer[1];

    LOGD("=== Main done!");
    return 0;
}

