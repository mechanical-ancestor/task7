#include"iostream"
#include"auto_aim/auto_aim.hpp"

int main(){
cv::VideoCapture cap(0,cv::CAP_V4L2); //尺寸为640*480

auto_aim::auto_aim auto_aim_cp;

std::cout<<"电脑摄像头创建成功！"<<std::endl;

//std::cout<<"打开电脑摄像头！"<<std::endl;

cv::Mat frame;

while(true){

  cap>>frame;

  auto_aim_cp.procces(frame);

  cv::imshow("aim_auto_cp",frame);
  if(cv::waitKey(30)==27){
    std::cout<<"手动退出成功！"<<std::endl;
   }
  }
  cap.release();
  cv::destroyAllWindows();
  return 0;
}