


#include <iostream>
#include <chrono>
#include <thread>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class RealtimeCamera {
private:
    void* handle_;
    bool is_grabbing_;
    
public:
    RealtimeCamera() : handle_(nullptr), is_grabbing_(false) {}
    
    ~RealtimeCamera() {
        stop();
    }
    
    // 打开相机
    bool open() {
        // 1. 枚举设备
        MV_CC_DEVICE_INFO_LIST device_list = {0};
        MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        
        if (device_list.nDeviceNum == 0) {
            return false;
        }
        
        
        // 2. 创建设备句柄（选择第一个设备）
        MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
        
        // 3. 打开设备
        MV_CC_OpenDevice(handle_);
        
        int ret = MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_Mono8);
        return true;
    }
    

    
    // 开启相机来截图
    bool start() {
        MV_CC_StartGrabbing(handle_);
        is_grabbing_ = true;
        return true;
    }
    
    // 获取一帧图像
    Mat getFrame() {
        if (!is_grabbing_) {
            return cv::Mat();
        }
        
        MV_FRAME_OUT_INFO_EX frame_info = {0};
        unsigned char* raw_data = new unsigned char[MV_ALG_E_DATA_SIZE];
        
        int ret = MV_CC_GetOneFrameTimeout(handle_, raw_data, MV_ALG_E_DATA_SIZE, 
                                          &frame_info, 100);
        
        cv::Mat image;
       if (ret == MV_OK) {
            if (frame_info.enPixelType == PixelType_Gvsp_Mono8) {
                // 灰度图像
                image = Mat(frame_info.nHeight, frame_info.nWidth, 
                               CV_8UC1, raw_data).clone();
            }
        }
        
        
        delete[] raw_data;
        return image;
    }
    
    // 关闭相机
    void stop() {
        if (is_grabbing_ && handle_) {
            MV_CC_StopGrabbing(handle_);
            is_grabbing_ = false;
        }
        
        if (handle_) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        
        
    }
};

int main() {
    std::cout << "=== 海康相机实时显示程序 ===" << std::endl;
    std::cout << "按以下键控制:" << std::endl;
    std::cout << "  ESC - 退出程序" << std::endl;
    std::cout << "  S   - 保存当前帧" << std::endl;
   
    
    RealtimeCamera camera;
    
    try {
        // 1. 打开相机
        if (!camera.open()) {
            return -1;
        }
        
        
        
        // 3. 开始采集
        if (!camera.start()) {
            return -1;
        }
        
        // 4. 创建显示窗口
        cv::namedWindow("海康工业相机画面", cv::WINDOW_AUTOSIZE);
        
        // 5. 主循环
        int frame_count = 0;
        int save_count = 0;
        auto last_time = std::chrono::high_resolution_clock::now();
        
        while (true) {
            // 获取一帧
            cv::Mat frame = camera.getFrame();
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            frame_count++;
            
            
            // 显示图像尺寸
            std::string size_text = "Size: " + std::to_string(frame.cols) + 
                                   "x" + std::to_string(frame.rows);
            cv::putText(frame, size_text, cv::Point(10, 110),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            // 显示图像
            cv::imshow("海康工业相机画面", frame);
            
            // 按键处理
            int key = cv::waitKey(1);
            if (key == 27) {  // ESC键退出
                break;
            }
            else if (key == 's' || key == 'S') {
                // 保存当前帧
                std::string filename = "img" + std::to_string(save_count++) + ".jpg";
                cv::imwrite(filename, frame);
                std::cout << "截图已保存: " << filename << std::endl;
                
                // 显示保存提示
                cv::putText(frame, "Saved!", cv::Point(frame.cols/2 - 50, 50),
                           cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
                cv::imshow("海康工业相机画面", frame);
                cv::waitKey(500);  // 显示500ms
            }
            
        }
        
        // 6. 清理
        cv::destroyAllWindows();
        camera.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "有错误" << e.what() << std::endl;
        camera.stop();
        return -1;
    }
    
    return 0;
}
