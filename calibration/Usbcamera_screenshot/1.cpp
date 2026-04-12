#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap(0); // 0 = 电脑默认摄像头
    if (!cap.isOpened())
    {
        cout << "无法打开摄像头" << endl;
        return -1;
    }

    namedWindow("Camera", WINDOW_NORMAL);
    Mat frame;
    int save_idx = 0;

    cout << "按 S 保存图片" << endl;
    cout << "按 ESC 退出" << endl;

    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        imshow("Camera", frame);

        int key = waitKey(1);
        if (key == 27) // ESC 退出
        {
            break;
        }
        else if (key == 's' || key == 'S') // 保存
        {
            string filename = "capture_" + to_string(save_idx++) + ".jpg";
            imwrite(filename, frame);
            cout << "已保存：" << filename << endl;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}