#include <iostream>
#include <opencv2/opencv.hpp>

#include "detector.hpp"

using namespace std;
using namespace cv;
using namespace auto_aim;

int main()
{
    // ========== 1. 构造 Detector 参数 ==========
    Detector::Params params;

    // ---- preprocess ----
    params.prepc_params.binary_threshold = 140;
    params.prepc_params.enemy_color = EnemyColor::UNKNOWN;

    // ---- light ----
    params.light_params.min_light_ratio  = 4.0f;
    params.light_params.max_light_ratio  = 20.0f;
    params.light_params.max_light_angle  = 30.0f;

    // ---- armor ----
    params.armor_params.min_small_center_dist = 2.0f;
    params.armor_params.max_small_center_dist = 4.0f;

    params.armor_params.min_large_center_dist = 4.0f;
    params.armor_params.max_large_center_dist = 10.0f;
    
    params.armor_params.max_angle_diff        = 15.0f;

    Detector detector(params);

    // ========== 2. 打开相机 ==========
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "[ERROR] camera open failed\n";
        return -1;
    }

    namedWindow("detector", WINDOW_AUTOSIZE);

    Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "[WARN] empty frame\n";
            continue;
        }

        // ========== 3. 只调用 detect ==========
        auto armors = detector.detect(frame);

        // ========== 4. 画调试 ==========
        detector.drawDebug(frame, Scalar(0,255,0));

        // 显示检测到的装甲数量
        putText(frame,
                "armors: " + to_string(armors.size()),
                Point(20, 30),
                FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar(0, 255, 0),
                2);

        imshow("detector", frame);

        char key = (char)waitKey(1);
        if (key == 27 || key == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
