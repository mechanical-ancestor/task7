#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/usbcamera/usbcamera.hpp"
#include "io/serials/serial.hpp"
#include "auto_aim/include/detector.hpp"
#include "auto_aim/include/solver.hpp"
#include "auto_aim/include/tracker.hpp"
#include "auto_aim/include/type.hpp"

using namespace std;
using namespace cv;
using namespace io;
using namespace auto_aim;


int main() {
    // ================ 1. 初始化 ================ //
    
    // ----- 1.1 USBCamera ----- //
    USBCamera camera(0);
    camera.open();
    cout << "[INFO] Camera   initialized successfully!" << endl;
    
    // ----- 1.2 Detector ----- //
    Detector::Params params;
    string params_path = "../configs/auto_aim_params.yaml";
    params.loadParams(params_path);
    Detector detector(params);
    cout << "[INFO] Detector initialized successfully!" << endl;

    // ----- 1.3 Solver ----- //
    string calib_path = "../configs/camera_calib.yaml";
    Solver solver(calib_path);
    cout << "[INFO] Solver   initialized successfully!" << endl;

    // ----- 1.4 Tracker ----- //
    ArmorTracker tracker;
    cout << "[INFO] Tracker initialized." << endl;

    // ----- 1.5 Serials ----- //
    string serial_port = "/dev/ttyUSB0";  // 串口设备路径
    int baudrate = 115200;                // 波特率
    Serial serial(serial_port, baudrate);
    if (serial.open()) {
        cout << "[INFO] Serial port " << serial_port << " opened successfully!" << endl;
    } else {
        cerr << "[ERROR] Serial port open failed!" << endl;
        return -1;
    }

    // =============== 2. 主程序 =============== //
    Mat frame;
    while(true) {
        if(!camera.read(frame)){
            cerr << "[WARN] Empty frame, continue..." << endl;
            continue;
        }

        // ----- 2.1 检测装甲板 ----- //
        vector<Armor> armors = detector.detect(frame);
        
        // ----- 2.2 位姿解算 ----- //
        int solved_count = 0;
        Armor* best_armor = nullptr;
        for(Armor& armor : armors)  { 
            if(!solver.solve(armor)) continue;
            ++solved_count;
            
            if(!best_armor || best_armor->position.y > armor.position.y){
                best_armor = &armor;
            } 
        }
        // ----- 2.3 卡尔曼预测 ----- //
        if(best_armor != nullptr) {
            auto current_time = std::chrono::steady_clock::now();
            tracker.update(best_armor->position, current_time); // updata
        }
        double dt = 0.03; // 30 ms
        tracker.predict(dt); // 预测

        // --- 2.4 显示结果 --- // 
        detector.drawDebug(frame, Scalar(0,255,0));
        solver.printSolverDebug(frame, armors);
        tracker.drawPredict(frame, 
                            solver.getCameraMatrix(), 
                            solver.getDistCoeffs(), 
                            Scalar(0,0,255)
                           );
        
        string text = "Detected: " + to_string(armors.size()) +
                " | Solved: " + to_string(solved_count);
        putText(frame, text, 
                Point(20, 20), 
                FONT_HERSHEY_SIMPLEX, 
                0.8, Scalar(0, 255, 255), 2);
        imshow("RESULT", frame);

        // --- 2.5 串口发送数据 --- //
        if (serial.isOpened() && best_armor != nullptr) {
            // 打包数据为字符串
            string send_str = "SOLVED:" + to_string(best_armor->position.x) + ","
                                        + to_string(best_armor->position.y) + ","
                                        + to_string(best_armor->position.z) + ")\n"
                              "PRED:" + to_string(tracker.getPrePosition().x) + ","
                                      + to_string(tracker.getPrePosition().y) + ","
                                      + to_string(tracker.getPrePosition().z) + ")\n";
            
            // 转换为uint8_t*并发送
            const uint8_t* data = reinterpret_cast<const uint8_t*>(send_str.c_str());
            serial.send(data, send_str.size());
        }

        // --- 2.6 退出 --- //
        char key = (char)waitKey(30);
        if (key == 27 || key == 'q') {
            cout << "[INFO] User exited program." << endl;
            break;
        }
    }

    return 0;
}