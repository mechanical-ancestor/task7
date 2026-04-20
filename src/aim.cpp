#include "MvCameraControl.h"
#include "yolov8.hpp"
#include "camera.hpp"
#include "target.hpp"
#include "armor.hpp"
#include "serial.hpp"
#include "Usbcamera.hpp"  

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <string>
#include <cstdio>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <csignal>

volatile sig_atomic_t g_stop_flag = 0;

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    Mat right_frame;
    int frame_count = 0;
    double dt = 0.033;
    target kalman(dt);
    cv::Point2f detect_point;
    // 串口配置
   const char* serial_port = "/dev/ttyUSB0"; // 替换为实际的
    speed_t baud_rate = B115200; // 必须波特率一致（接收端用什么，这里就用什么）speed_t 波特率专用
    int fd;
    serial my_serial;
    my_serial.serial_begin(serial_port, baud_rate, fd);
    try {
        std::string yolov8_cfg = "/home/wrt/task_7/configs/yolov8.yaml";
        std::string camera_cfg = "/home/wrt/task_7/configs/camera_calibration_sources.yaml";
        
       
       
        yolov8 detector_(yolov8_cfg);
      

        UsbCamera cap;  // 相机类

        // 加载相机标定
        cap.loadCalibParams(camera_cfg);

        // 打开相机
        if (!cap.open(0)) {
            return -1;
        }
       
        // 设分辨率
        cap.cap_.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap.cap_.set(CAP_PROP_FRAME_HEIGHT, 720);
      
       // namedWindow("Original Camera", WINDOW_NORMAL);
        namedWindow("Detection Result", WINDOW_NORMAL);
        
        Mat frame;
        
        while (true) {
            
            frame = cap.getFrame();
           //if (frame.empty()) {
           //     this_thread::sleep_for(chrono::milliseconds(10));
           //     continue;
           // }

            // 图像去畸变
           undistort(frame, right_frame, cap.camera_matrix, cap.dist_coeffs);

            //imshow("Original Camera", frame);
            
            frame_count++;
            
            try {
                list<Armor> armors = detector_.detect(right_frame, frame_count);
                detector_.draw_detections(right_frame, armors, frame_count);
                
                if (!armors.empty()) {
                    Armor target_armor = *armors.begin();
                    detect_point.x = target_armor.x + target_armor.width / 2.0f;
                    detect_point.y = target_armor.y + target_armor.height / 2.0f;
                    
                    int predict_x_= static_cast<int>(detect_point.x - armor_w / 2.0f);
                    int predict_y_= static_cast<int>(detect_point.y - armor_h / 2.0f);
                    Rect a(predict_x_,predict_y_,armor_w, armor_h);
                    rectangle(right_frame, , Scalar(0, 0, 255), 2);

                    Mat predicted_state = kalman.predict();
                    kalman.correct(detect_point);
                     // 预测效果绘制
                    float predict_x = predicted_state.at<float>(0);
                    float predict_y = predicted_state.at<float>(2);
                    circle(right_frame, detect_point, 6, Scalar(0, 0, 255), -1);// 红色：装甲板中心 半径6 实心圆
                    circle(right_frame, Point2f(predict_x, predict_y), 6, Scalar(0, 255, 0), -1); // 绿色：预测中心
                
                    // 串口发数据
                    my_serial.send_predict(predict_x, predict_y);

                    int armor_w = target_armor.width;
                    int armor_h = target_armor.height;
                    // 反向计算预测框的左上角坐标
                    int predict_x_left = static_cast<int>(predict_x - armor_w / 2.0f);
                    int predict_y_top = static_cast<int>(predict_y - armor_h / 2.0f);
                    // 绘制预测的装甲板矩形框（绿色）
                    Rect predict_armor_rect(predict_x_left, predict_y_top, armor_w, armor_h);
                    rectangle(right_frame, predict_armor_rect, Scalar(0, 255, 0), 2);
                    
                 }
            } catch (const std::exception& e) {
                cerr << "Detection error: " << e.what() << endl;
            }
            
            imshow("Detection Result", right_frame);
            
            int key = waitKey(30);
            if (key == 27) {
                cout << "按ESC退出" << endl;
                break;
            }
           
        }
        
        destroyAllWindows();
        cap.close();//关相机
        my_serial.serial_close();// 关闭串口，释放资源
        
    } catch (const cv::Exception& e) {
        cerr << "OpenCV Exception: " << e.what() << endl;
        my_serial.serial_close();
        return -1;
    } catch (const std::exception& e) {
        cerr << "Standard Exception: " << e.what() << endl;
        my_serial.serial_close();
        return -1;
    }
    
   
    return 0;
}