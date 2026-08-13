// >> Team 3 中线检测 — ResNet18 BPU 推理回归赛道中心线坐标
#include "centerline_percept/centerline_detector.h"

#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"

namespace centerline_percept {

constexpr int IMG_WIDTH = 640;
constexpr int IMG_HEIGHT = 480;

constexpr int CROP_LEFT = 0;
constexpr int CROP_RIGHT = 640 - 1;
constexpr int CROP_TOP = 130 - 1;
constexpr int CROP_BOTTOM = 480 - 1;

void build_nv12_tensor(void *img_data,
                       int height,
                       int width,
                       hbDNNTensor *tensor) {
  auto &props = tensor->properties;
  props.tensorType = HB_DNN_IMG_TYPE_NV12;
  props.tensorLayout = HB_DNN_LAYOUT_NCHW;
  auto &vs = props.validShape;
  vs.numDimensions = 4;
  vs.dimensionSize[0] = 1;
  vs.dimensionSize[1] = 3;
  vs.dimensionSize[2] = height;
  vs.dimensionSize[3] = width;

  auto &as = props.alignedShape;
  as = vs;

  int32_t len = height * width * 3 / 2;
  hbSysAllocCachedMem(&tensor->sysMem[0], len);
  memcpy(tensor->sysMem[0].virAddr, img_data, len);
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
}

void prepare_empty_tensor(int height,
                          int width,
                          hbDNNTensor *tensor) {
  auto &props = tensor->properties;
  props.tensorType = HB_DNN_IMG_TYPE_NV12;
  props.tensorLayout = HB_DNN_LAYOUT_NCHW;

  auto &vs = props.validShape;
  vs.numDimensions = 4;
  vs.dimensionSize[0] = 1;
  vs.dimensionSize[1] = 3;
  vs.dimensionSize[2] = height;
  vs.dimensionSize[3] = width;

  auto &as = props.alignedShape;
  int32_t stride = ALIGN_16(width);
  as.numDimensions = 4;
  as.dimensionSize[0] = 1;
  as.dimensionSize[1] = 3;
  as.dimensionSize[2] = height;
  as.dimensionSize[3] = stride;

  int32_t len = height * stride * 3 / 2;
  hbSysAllocCachedMem(&tensor->sysMem[0], len);
}

CenterlineDetector::CenterlineDetector(const std::string& node_name,
                                         const NodeOptions& options)
  : DnnNode(node_name, options) {
  this->declare_parameter<std::string>("model_path", model_path_);
  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->declare_parameter<std::string>("mode_name", mode_name_);

  this->get_parameter("model_path", model_path_);
  this->get_parameter("sub_img_topic", sub_img_topic_);
  this->get_parameter("mode_name", mode_name_);

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CenterlineDetector"), "Node initialized failed!");
  }

  rclcpp::QoS qos(5);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  publisher_ =
    this->create_publisher<ai_msgs::msg::PerceptionTargets>("/lane_center", 5);
  hbmem_sub_ =
    this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      sub_img_topic_,
      qos,
      std::bind(&CenterlineDetector::image_callback,
      this,
      std::placeholders::_1));

  rclcpp::QoS mode_qos(rclcpp::KeepLast(1));
  mode_qos.reliable();
  mode_qos.transient_local();
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/track_switch",
      mode_qos,
      std::bind(&CenterlineDetector::mode_switch_callback,
                this,
                std::placeholders::_1));
}

CenterlineDetector::~CenterlineDetector() {
}

int CenterlineDetector::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("CenterlineDetector"), "model file path: %s", model_path_.c_str());
  dnn_node_para_ptr_->model_file = model_path_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int CenterlineDetector::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &outputs) {
  auto parser = std::make_shared<LanePointParser>();
  auto result = std::make_shared<LanePointResult>();
  parser->Parse(result, outputs->output_tensors[0]);
  float x = result->x;
  float y = result->y;
  RCLCPP_INFO(rclcpp::get_logger("CenterlineDetector"),
               "post result x: %d    y: %d", int(x), int(y));
  ai_msgs::msg::PerceptionTargets::UniquePtr msg(
        new ai_msgs::msg::PerceptionTargets());
  msg->set__header(*outputs->msg_header);
  ai_msgs::msg::Target target;
  target.set__type("lane_center");
  ai_msgs::msg::Point center_pt;

  geometry_msgs::msg::Point32 pt;
  pt.set__x(x);
  pt.set__y(y);
  center_pt.point.emplace_back(pt);
  center_pt.point.emplace_back(pt);
  std::vector<ai_msgs::msg::Point> pts;
  pts.push_back(center_pt);
  target.set__points(pts);
  msg->targets.emplace_back(target);
  publisher_->publish(std::move(msg));
  return 0;
}

void CenterlineDetector::mode_switch_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const bool enabled = msg->data == mode_name_;
  const bool prev = infer_enabled_.exchange(enabled);
  if (prev != enabled) {
    RCLCPP_WARN(this->get_logger(),
                "Inference %s (mode=%s, request=%s)",
                enabled ? "ON" : "OFF",
                mode_name_.c_str(),
                msg->data.c_str());
  }
}

void CenterlineDetector::image_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }
  if (!infer_enabled_.load()) {
    return;
  }
  std::stringstream ss;
  ss << "Received img encoding: "
     << std::string(reinterpret_cast<const char*>(msg->encoding.data()))
     << ", h: " << msg->height << ", w: " << msg->width
     << ", step: " << msg->step << ", index: " << msg->index
     << ", stamp: " << msg->time_stamp.sec << "_"
     << msg->time_stamp.nanosec << ", data size: " << msg->data_size;
  RCLCPP_DEBUG(rclcpp::get_logger("CenterlineDetector"), "%s", ss.str().c_str());

  auto model_mgr = GetModel();
  if (!model_mgr) {
    RCLCPP_ERROR(rclcpp::get_logger("CenterlineDetector"), "Invalid model handle");
    return;
  }

  hbDNNRoi roi;
  roi.left = CROP_LEFT;
  roi.top = CROP_TOP;
  roi.right = CROP_RIGHT - 1;
  roi.bottom = CROP_BOTTOM - 1;

  cv::Mat img_mat(msg->height * 3 / 2, msg->width, CV_8UC1, (void*)(msg->data.data()));
  cv::Range row_range(CROP_TOP, CROP_BOTTOM);
  cv::Range col_range(CROP_LEFT, CROP_RIGHT);
  cv::Mat crop_mat = hobot_cv::hobotcv_crop(img_mat, msg->height, msg->width, 224, 224, row_range, col_range);

  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(crop_mat.data),
      224,
      224,
      224,
      224);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("CenterlineDetector"), "Pyramid creation failed");
    return;
  }

  std::vector<std::shared_ptr<DNNInput>> inputs;
  auto rois = std::make_shared<std::vector<hbDNNRoi>>();
  roi.left = 0;
  roi.top = 0;
  roi.right = 224;
  roi.bottom = 224;
  rois->push_back(roi);

  for (size_t i = 0; i < rois->size(); i++) {
    for (int32_t j = 0; j < model_mgr->GetInputCount(); j++) {
      inputs.push_back(pyramid);
    }
  }

  auto dnn_output = std::make_shared<DnnNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(msg->index));
  dnn_output->msg_header->set__stamp(msg->time_stamp);
  Predict(inputs, dnn_output, rois);
}

int CenterlineDetector::Predict(
  std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
  const std::shared_ptr<DnnNodeOutput> &output,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  RCLCPP_INFO(rclcpp::get_logger("CenterlineDetector"), "input size:%d roi size:%d", dnn_inputs.size(), rois->size());
  return Run(dnn_inputs,
             output,
             rois,
             false);
}

int32_t LanePointParser::Parse(
    std::shared_ptr<LanePointResult> &output,
    std::shared_ptr<DNNTensor> &output_tensor) {
  if (!output_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("CenterlineDetector"), "invalid tensor");
    rclcpp::shutdown();
  }
  std::shared_ptr<LanePointResult> result;
  if (!output) {
    result = std::make_shared<LanePointResult>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<LanePointResult>(output);
  }
  DNNTensor &tensor = *output_tensor;
  const int32_t *shape = tensor.properties.validShape.dimensionSize;
  RCLCPP_DEBUG(rclcpp::get_logger("CenterlineDetector"),
               "shape[1]: %d shape[2]: %d shape[3]: %d",
               shape[1],
               shape[2],
               shape[3]);
  hbSysFlushMem(&(tensor.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float raw_x = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[0];
  float raw_y = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[1];

  result->x = raw_x * (CROP_RIGHT - CROP_LEFT) + CROP_LEFT;
  result->y = raw_y * (CROP_BOTTOM - CROP_TOP) + CROP_TOP;

  RCLCPP_INFO(rclcpp::get_logger("CenterlineDetector"),
               "raw: %f, %f -> mapped: %f, %f", raw_x, raw_y, result->x, result->y);
  return 0;
}

}  // namespace centerline_percept

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<centerline_percept::CenterlineDetector>("centerline_detector"));
  rclcpp::shutdown();
  RCLCPP_WARN(rclcpp::get_logger("CenterlineDetector"), "Centerline detector exited.");
  return 0;
}
