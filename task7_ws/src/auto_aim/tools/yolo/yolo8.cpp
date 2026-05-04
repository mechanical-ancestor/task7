#include"tools/yolo8.hpp"

namespace tools{


    YOLO8::YOLO8(const std::string &model_path,float conf_threashold,float iou_threshold){
       this->model_path=model_path;
       this->conf_threshold=conf_threashold;
       this->iou_shreshold=iou_threshold;
       this->infer_request=Preprocces(model_path);
    };

    ov::InferRequest YOLO8::Preprocces(std::string model_path){
        ov::Core core;
        auto model=core.read_model(model_path);
        //创建官方预处理模型，并定义图像输出格式(640*640)
        ov::preprocess::PrePostProcessor ppp(model);

        auto &input_info=ppp.input();
        //声明传入传入原始图像的格式
        input_info.tensor()
              .set_element_type(ov::element::u8)                           //int8
              .set_layout("NHWC")                                          //布局为通道，高，宽，批量
              .set_color_format(ov::preprocess::ColorFormat::BGR);          //颜色为BGR
        
        //官方预处理
        input_info.preprocess()
                .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)   // 缩放
                .convert_color(ov::preprocess::ColorFormat::RGB)           // BGR->RGB
                .convert_element_type(ov::element::f32)                   // 转 float
                .scale(255.0f);                                            //归一化
        // 模型期望输入是 NCHW
        input_info.model().set_layout("NCHW");

        //构建带预处理的模型
        model= ppp.build();
        
        auto compiled_model=core.compile_model(model,"CPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        );

        return compiled_model.create_infer_request();
    };

    ov::Tensor YOLO8::letterbox(cv::Mat image){
        int model_h=640;
        int model_w=640;
        float scale=std::min((float)model_w/image.cols,(float)model_h/image.rows);
        float img_h=round(image.rows*scale);
        float img_w=round(image.cols*scale);

        cv::resize(image, image, cv::Size(img_w, img_h), 0, 0, cv::INTER_LINEAR);

        int h=(model_h-img_h)/2;
        int w=(model_w-img_w)/2;
        //填充
        cv::copyMakeBorder(image, image, h, h, w, w, cv::BORDER_CONSTANT, cv::Scalar(114,114,114));

        ov::Tensor tensor(ov::element::u8, {1, (size_t)image.rows, (size_t)image.cols, 3},image.data);
        //std::memcpy(tensor.data(), image.data, tensor.get_byte_size());
        
        return tensor;
    };

   //推理函数
    ov::Tensor YOLO8::infer(ov::Tensor &pre_image){
        infer_request.set_input_tensor(pre_image);
        infer_request.infer();
        return infer_request.get_output_tensor();
    };
    
    //置信度过滤+坐标系转换
    std::vector<Armor> YOLO8::Postprocess(const ov::Tensor &output_tensor,int &img_h, int &img_w) {
    //获取形状张量
    auto output_shape=output_tensor.get_shape();
    //构造Mat向量,output_shape[1]包含坐标，置信度等信息，output_shape[2]包含框数
    cv::Mat output(output_shape[1],output_shape[2],CV_32F,output_tensor.data()); //8400列，5行
    //原本列是框数，行是信息
    //转换col(列)和row（行）的位置
    //std::cout<<"前，列："<<output.cols<<std::endl;
    // std::cout<<"行:"<<output.rows<<std::endl;
    cv::transpose(output,output);
    //行是框数，列是信息
    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    int model_w = 640;
    int model_h = 640;

    float scale = std::min((float)model_w / img_w, (float)model_h / img_h);
    float scaled_w = img_w * scale;
    float scaled_h = img_h * scale;

    cv::Point2f centry;

    // std::cout<<"列："<<output.cols<<std::endl;
    // std::cout<<"行:"<<output.rows<<std::endl;
//置信度过滤
    for(int r=0;r<output.rows;r++){
       auto xywh=output.row(r).colRange(0,4);         //get x,y,w,h
       auto scores =output.row(r).colRange(4,5);      //get confidence
       
      // auto one_key_points=output.row(r).colRange(5,14);

       std::vector<cv::Point2f> Armor_kry_points;

       double score;
       cv::Point max_point;
        
       cv::minMaxLoc(scores,nullptr,&score,nullptr,&max_point);
      
        //左上角为坐标原点,opencv和opcnVINO同理
       if(score<conf_threshold) continue;

       auto x=xywh.at<float>(0);
       auto y=xywh.at<float>(1);
       auto w=xywh.at<float>(2);
       auto h=xywh.at<float>(3);
       //坐标缩放
       auto left=static_cast<int>((x-0.5*w)/scale);
       auto top=static_cast<int>((y-h)/scale);
       auto width=static_cast<int>(w/scale);
       auto height=static_cast<int>(h/scale);
       //std::cout<<"置信度："<<scores<<std::endl;
       centry.x=x/scale;
       centry.y=(y-0.5*h)/scale;

       confidences.emplace_back(score);
       boxes.emplace_back(left,top,width,height);
    }
    std::vector<int> indis;
    //NMS抑制
    cv::dnn::NMSBoxes(boxes,confidences,conf_threshold,iou_shreshold,indis);

    std::vector<Armor> Armors;
    cv::Point2f Points;

    for(auto i:indis){
        Armors.emplace_back(boxes[i],confidences[i]);
    }
    return Armors;
    }
    
    //画框函数
    void  YOLO8::drawContours(cv::Mat &image,const std::vector<Armor> &armors_List){
        for(const auto &armors:armors_List){
        //画框
        cv::rectangle(image,armors.box,cv::Scalar(0,0,255),5);
        //拼接置信度
        char text[32];
        sprintf(text,"Armor %0.2f",armors.confidence);
        //物体类别
       
        // 文字位置：框左上角稍微往上一点
        cv::putText(image, text, 
                    cv::Point(armors.box.x, armors.box.y - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.5,                    // 字体大小
                    cv::Scalar(255,0,0),   
                    2);                     // 线宽
      }

    }

    void YOLO8::drawContours(cv::Mat &image,const tools::Armor &armor){      //画框函数

      //左上角为坐标原点,box的x，y为左上角坐标
      cv::Rect rect;

      rect.x=armor.Predict_Points[0].x;    //左上角的角点即为rext需要的左上角坐标
      rect.y=armor.Predict_Points[0].y;

      rect.width=armor.box.width;
      rect.height=armor.box.height;
      cv::rectangle(image,rect,cv::Scalar(255,0,0),4);   //蓝色的框，线宽为4

      char text[32];
      sprintf(text,"predict");
      //物体类别
      // 文字位置：框左上角稍微往上一点
      cv::putText(image, text, 
                  cv::Point(rect.x, rect.y - 5), 
                  cv::FONT_HERSHEY_SIMPLEX, 
                  0.5,                    // 字体大小
                  cv::Scalar(0,0,255),   
                  2);                     // 线宽
    }
   
    std::vector<Armor> YOLO8::solver(const cv::Mat &image){
        int img_h=image.rows;
        int img_w=image.cols;
        auto pre_image=letterbox(image);
        ov::Tensor output_tensor=infer(pre_image);
        auto armors_List=Postprocess(output_tensor,img_h,img_w);
        return armors_List;
    };

}