#ifndef CENTERLINE_PERCEPT__CENTERLINE_DETECTOR_H
#define CENTERLINE_PERCEPT__CENTERLINE_DETECTOR_H

#include <atomic>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "sensor_msgs/msg/image.hpp"

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::DNNTensor;

namespace centerline_percept {

class LanePointResult {
 public:
  float x;
  float y;
  void Reset() {x = -1.0; y = -1.0;}
};

class LanePointParser {
 public:
  LanePointParser() {}
  ~LanePointParser() {}
  int32_t Parse(
      std::shared_ptr<LanePointResult>& output,
      std::shared_ptr<DNNTensor>& output_tensor);
};

class CenterlineDetector : public DnnNode {
 public:
  CenterlineDetector(const std::string& node_name,
                        const NodeOptions &options = NodeOptions());
  ~CenterlineDetector() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Predict(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
              const std::shared_ptr<DnnNodeOutput> &output,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois);
  void image_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  void mode_switch_callback(const std_msgs::msg::String::SharedPtr msg);
  bool GetParams();
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
    hbmem_sub_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    mode_sub_ = nullptr;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher_ =
      nullptr;
  cv::Mat image_bgr_;
  std::string model_path_ = "config/charlie_channel_center.bin";
  std::string sub_img_topic_ = "/raw_video";
  std::string mode_name_ = "disabled";
  std::atomic<bool> infer_enabled_{false};
};

}  // namespace centerline_percept

#endif  // CENTERLINE_PERCEPT__CENTERLINE_DETECTOR_H
