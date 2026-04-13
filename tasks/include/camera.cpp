#include "camera.hpp"
#include "MvCameraControl.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;




   RealtimeCamera::RealtimeCamera() 
    {
        handle_=nullptr;     //先初始化为空指针
         is_collect=false;
    }
    
    RealtimeCamera::~RealtimeCamera() {
        stop();
    }
    
    // 打开相机

    bool RealtimeCamera::open() {
        MV_CC_DEVICE_INFO_LIST device_list = {0};
        int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (ret != MV_OK || device_list.nDeviceNum == 0) {
            return false;
        }

        
        
        // 创建设备句柄（选择第一个设备）
        ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
        if (ret != MV_OK) {
            return false;
        }
        
        // 打开设备
        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        // 设模式为连续采集
        ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        if (ret != MV_OK) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }
        
        
        return true;
    }
    
    // 开始显示
    bool RealtimeCamera::start() {
    if (handle_ == nullptr) {
        return false;
    }

        MV_CC_StartGrabbing(handle_);
        is_collect = true;
        return true;
    }
    
    // 获取一帧图像
    Mat RealtimeCamera::getFrame() {
        if (!is_collect) {
            return Mat();
        }
        
        MV_FRAME_OUT_INFO_EX frame_info = {0};
        unsigned char* raw_data = new unsigned char[MV_ALG_E_DATA_SIZE];
        
        int ret = MV_CC_GetOneFrameTimeout(handle_, raw_data, MV_ALG_E_DATA_SIZE, 
                                          &frame_info, 100);
        //转BGR
        Mat image;
        Mat bgr_img;
        if (ret == MV_OK) {
            if (frame_info.enPixelType == PixelType_Gvsp_Mono8) {
                // 灰度图像
                image = Mat(frame_info.nHeight, frame_info.nWidth, 
                               CV_8UC1, raw_data).clone();
                 cvtColor(image, bgr_img, COLOR_GRAY2BGR); // 单通道转三通道BGR
                  
            }
            else if (frame_info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                // RGB彩色图像
                image = Mat(frame_info.nHeight, frame_info.nWidth, 
                               CV_8UC3, raw_data).clone();
                cvtColor(image, bgr_img, COLOR_RGB2BGR);//转BGR
                
            }
            else if (frame_info.enPixelType == PixelType_Gvsp_BayerRG8) {
                // Bayer格式
                Mat bayer_img(frame_info.nHeight, frame_info.nWidth, 
                                 CV_8UC1, raw_data);
                cvtColor(bayer_img, bgr_img, COLOR_BayerRG2BGR);//转BGR
               
            }
        }
        
        delete[] raw_data;
        return bgr_img;
    }
    
    // 关闭相机
    void RealtimeCamera::stop() {
        if (is_collect && handle_) {
            MV_CC_StopGrabbing(handle_);
            is_collect = false;
        }
        
        if (handle_) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        
        cout << "相机关闭" << endl;
    }

// 加载相机内参等
bool RealtimeCamera::loadCameraCalibParams(const std::string& camera_sources_path) {
    FileStorage fs(camera_sources_path, FileStorage::READ);
    if (!fs.isOpened()) return false;
    
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    
    return !camera_matrix.empty() && !dist_coeffs.empty();
}





