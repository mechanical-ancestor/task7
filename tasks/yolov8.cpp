
#include "yolov8.hpp"
#include "armor.hpp"
#include "target.hpp"

#include <list>
#include <iostream>
//#include <fmt/chrono.h> //保存文件类的工具
#include <yaml-cpp/yaml.h> //配置文件相关类
#include <algorithm> //算法
#include <filesystem> //用于创建文件目录
#include <opencv2/dnn.hpp>



//namespace aim{  //定义一个类似于文件夹的空间，防止与其他变量名冲突
yolov8::yolov8(const std::string & yolov8_yaml_path) //创建yolov8类的构造函数，初始化模型路径(类名与构造函数名必须一致！)
: detector_(yolov8_yaml_path)   // 初始化列表
{
// 加载YAML配置文件
  auto yaml = YAML::LoadFile(yolov8_yaml_path);  // 传入配置文件的路径 auto:让编译器自动推理返回的变量类型 简化代码

  
  model_path_ = yaml["yolov8_model_path"].as<std::string>();  // YOLOv8模型路径
  device_ = yaml["device"].as<std::string>();                // 推理设备的选择
  int min_confidence_= yaml["min_confidence"].as<double>();     // 装甲板最小置信度（低于则不画框，筛掉）
  //min_confidence_=con/10.0;
  // ROI：只检测该区域内的装甲板，减少计算量
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();          // ROI左上角x坐标
  y = yaml["roi"]["y"].as<int>();          // ROI左上角y坐标
  width = yaml["roi"]["width"].as<int>();  // ROI宽度（-1表示不裁剪，用图像宽度）
  height = yaml["roi"]["height"].as<int>();// ROI高度（～～～～～，用图像高度）
  use_roi = yaml["use_roi"].as<bool>();   // 是否启用ROI裁剪（true/false）
  roi_cut = cv::Rect(x, y, width, height);    // 用opencv的rect函数实现裁剪
  restore_= cv::Point2f(x, y);             // ROI偏移量（还原坐标时用）把裁剪后的图片还原到原来大小！！这个不能少不然瞄准会失效


   //加载YOLOv8模型并初始化OpenVINO预处理
  auto model = core_.read_model(model_path_);  // core_是OpenVINO的Core对象（加载模型）
  ov::preprocess::PrePostProcessor ppp(model); // 预处理/后处理配置器
  auto & input = ppp.input();  // 获取模型输入节点

  // 配置输入张量的属性（必须和模型输入匹配）
  input.tensor()
    .set_element_type(ov::element::u8)        // 输入数据类型（8位无符号整数，对应OpenCV的CV_8UC3）
    .set_shape({1, 416, 416, 3})              // 模型输入尺寸（NHWC：1张图、416高、416宽、3通道）
    .set_layout("NHWC")                       // 输入张量布局（OpenCV图像是NHWC：批次-高度-宽度-通道）
    .set_color_format(ov::preprocess::ColorFormat::BGR);  // 输入颜色格式（OpenCV默认BGR）

  input.model().set_layout("NCHW");  // 模型期望的布局（深度学习模型通常是NCHW：批次-通道-高度-宽度）

  // 配置输入预处理流程
  input.preprocess()
    .convert_element_type(ov::element::f32)  // 转换为浮点型（模型计算需要float32）
    .convert_color(ov::preprocess::ColorFormat::RGB)  // 转换为RGB（YOLOv8训练用RGB）
    .scale(255.0);  // 归一化系数（除以255，把0-255的像素值转成0-1） 最好别改了

  // 性能模式配置
  model = ppp.build();  // 构建预处理管道
  // 编译模型（加载到指定设备，并优化为低延迟模式）
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    // LATENCY（低延迟）适合装甲板的实时检测

  }   

    

     std::list<Armor>  yolov8::detect (const cv::Mat & raw_img,int frame_count){
       cv::Mat bgr_img;
  if(use_roi){
    if(roi_cut.width==-1){
      roi_cut.width=raw_img.cols;
    }
    if(roi_cut.height==-1){
      roi_cut.height=raw_img.rows;
    }
    bgr_img=raw_img(roi_cut);
  }else{
    bgr_img=raw_img;
  }
  auto x_scale = static_cast<double>(416) / bgr_img.rows;  // 高度缩放系数
  auto y_scale = static_cast<double>(416) / bgr_img.cols;  // 宽度缩放系数
  auto scale = std::min(x_scale, y_scale);  // 取最小系数（等比例缩放）
  // 计算缩放后的宽高
  auto hei = static_cast<int>(bgr_img.rows* scale);  //变量有点混乱
  auto wid = static_cast<int>(bgr_img.cols* scale);

  // 构造模型输入图像（416x416，red底填充）
  auto input = cv::Mat(416, 416, CV_8UC3, cv::Scalar(0, 0, 255));  // 填充色
  auto roi = cv::Rect(0, 0, wid, hei);  // 缩放后的图像贴到左上角
  cv::resize(bgr_img, input(roi), {wid, hei});  // 缩放图像并填充到input中

  // 构造OpenVINO输入张量（把OpenCV的Mat转成模型需要的格式）
  ov::Tensor input_tensor(ov::element::u8, {1, 416, 416, 3},input.data);

  // 模型推理
  auto infer_request = compiled_model_.create_infer_request();  // 创建推理请求
  infer_request.set_input_tensor(input_tensor);                 // 设置输入张量
  infer_request.infer();                                        // 执行推理（核心步骤）

  // 获取推理输出
  auto output_tensor = infer_request.get_output_tensor();  // 获取输出张量
  auto output_shape = output_tensor.get_shape();           // 获取输出形状（比如[1, 8400, 14]）
  // 转换为OpenCV的Mat（方便后续解析）：行数=输出维度1，列数=输出维度2，浮点型
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  // 后处理解析结果（调用parse函数）
  return parse(scale, output, raw_img, frame_count);

}



std::list<Armor> /*批量存储、传递多个装甲板 不能少不然无法正常对装甲板进行绘制*/yolov8::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 转置输出张量
  cv::transpose(output, output);

  // 定义存储结果的容器
  std::vector<float> confidences;        // 置信度（0-1）
  std::vector<cv::Rect> boxes;           // 检测框（左上x、左上y、宽、高）

  // 遍历每一行（每一行对应一个检测框）
  for (int r = 0; r < output.rows; r++) {
    // 提取该行的前4列：xywh（中心x、中心y、宽、高）
    auto xywh = output.row(r).colRange(0, 4);
    double confidence = output.row(r).at<float>(4);  // 直接提取置信度（第4列）
    if (confidence < min_confidence_) continue;  // 过滤低置信度框

    // 还原检测框坐标（从模型的416x416尺寸还原到原始图像尺寸）
    auto x = xywh.at<float>(0);    // 模型输出的中心x
    auto y = xywh.at<float>(1);    // 模型输出的中心y
    auto wid = xywh.at<float>(2);    // 模型输出的宽度
    auto hei = xywh.at<float>(3);    // 模型输出的高度
    // 转换为左上角坐标 + 宽高（并除以缩放系数scale）
    auto left = static_cast<int>((x - 0.5 * wid) / scale);
    auto top = static_cast<int>((y - 0.5 * hei) / scale);
    auto width = static_cast<int>(wid / scale);
    auto height = static_cast<int>(hei / scale);


    // 保存当前检测框的结果
    confidences.emplace_back(confidence);        // 置信度
    boxes.emplace_back(left, top, width, height);  // 检测框
  }

  // NMS非极大值抑制（过滤重叠的检测框）
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences,static_cast<float>(min_confidence_),0.3, indices);
  
 // 构造装甲板列表
  std::list<Armor> armors; //批量存储多个装甲板 便于绘制框
  for (const auto & i : indices) {
    // 还原ROI偏移（如果启用了ROI，坐标需要加上偏移量）
    if (use_roi) {
      armors.emplace_back(0, confidences[i], boxes[i], restore_); //armors 列表，在尾部新增一个对象 调用armor.hpp中的函数

    } else {
      armors.emplace_back(0, confidences[i], boxes[i]);
    }
  }
  return armors;
}
void yolov8::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{// 绘制ROI区域
   cv::Mat detection = img.clone();

  if (use_roi) {
    cv::Scalar red(0, 0, 255);  // 颜色
    cv::rectangle(detection, roi_cut, red, 3);  //画框 
  }
  cv::Scalar green(0, 255, 0); // 检测框颜色（绿色）
  for (const auto& armor : armors) {
    cv::rectangle(detection, armor.box, green, 2);
    // 绘制
    char conf_text[20];
    sprintf(conf_text, "Conf: %.2f", armor.confidence);
    cv::putText(detection, conf_text, armor.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1);
  }
  
  
    // 缩小图像（缩小一半）
  cv::resize(detection, detection, {}, 0.5, 0.5);
  // 显示图像
  cv::imshow("result", detection);
  cv::waitKey(1);
}
//}
