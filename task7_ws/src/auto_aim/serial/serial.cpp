#include"serial/serial.hpp"


namespace Serial{
    serial::serial(const std::string &_file,int baudrate):_file(_file),baudrate(baudrate) {
    fd = open(_file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "无法打开串口！" << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout<<"串口创建成功！"<<std::endl;
        std::cout<<"[INFO] 串口名："<<_file<<"  "<<"波特率："<<baudrate<<std::endl;
    }
    struct termios options;                //定义一个termios结构体变量，用于配置串口参数
    tcgetattr(fd, &options);               //获取当前串口配置
    cfsetispeed(&options, baudrate);       //设置输入波特率
    cfsetospeed(&options, baudrate);       //设置输出波特率
    options.c_cflag |= (CLOCAL | CREAD);   //本地连接，启用接收器
    options.c_cflag &= ~PARENB;            //无奇偶校验
    options.c_cflag &= ~CSTOPB;            //1位停止位
    options.c_cflag &= ~CSIZE;             //清除数据位设置
    options.c_cflag |= CS8;                //8位数据位
    tcsetattr(fd, TCSANOW, &options);      //将配置应用到串口
   } 
    
   serial::~serial() {
    if (fd != -1) {
        close(fd);                   //关闭串口文件描述符，释放资源
    }
   }
    void serial::write(){            //代表当前帧没有装甲板进行预测
        std::stringstream ss;
        ss<<"当前帧的观测结果："<<std::endl<<"空"<<std::endl<<"----"<<std::endl;
        std::string str =ss.str();
        if(fd!=-1){
            ssize_t ret=::write(fd,str.c_str(),str.size());
            if(ret==-1){
                std::cerr<<"write error"<<std::endl;
            }
        }
    }

    void serial::write(const std::vector<cv::Mat> &data){      //输出预测向量
        std::stringstream ss;
        std::string str;
        ss <<"当前帧的预测结果:"<<std::endl<<"旋转向量："<<std::endl<<data[0] <<std::endl<<"平移向量："<<std::endl<<data[1]<<std::endl;
            str=ss.str();
          if(fd != -1){
            ssize_t ret=::write(fd,str.c_str(),str.size());
          if(ret==-1){
            std::cerr<<"write error"<<std::endl;
          }
          }
    }

    void serial::write(const cv::Mat &rvec,const cv::Mat &tvec){
         std::stringstream ss;
         ss <<"当前帧的预测结果:"<<std::endl<<"旋转向量："<<std::endl<<rvec <<std::endl<<"平移向量："<<std::endl<<tvec<<std::endl<<"----"<<std::endl;
         std::string str=ss.str();
         if(fd!=-1){
            ssize_t ret=::write(fd,str.c_str(),str.size());
            if(ret==-1){
            std::cerr<<"write error"<<std::endl;
          }
         
        }
  }
  void serial::write(const cv::Point2f &centry){
     std::stringstream ss;
     ss << "当前帧预测结果："<<std::endl<<"中心坐标："<<centry<<std::endl<<"----"<<std::endl;
     std::string str=ss.str();
     if(fd!=-1){
        ssize_t ret=::write(fd,str.c_str(),str.size());
        if(ret==-1){
        std::cerr<<"write error"<<std::endl;
        }
     }
  }
}