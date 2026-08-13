// >> Team 3 码标解码器 — ZBar 二维码读取 + 方向编码(顺1/逆2)
#include <rclcpp/rclcpp.hpp>

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <string>
#include <stdexcept>


class BarcodeDecoder : public rclcpp::Node
{
public:
  BarcodeDecoder() : Node("barcode_decoder"), frame_count_(0), debounce_counter_(0), last_barcode_id_(0), last_barcode_raw_("")
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscriber_hbmem_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/nv12_img",
            qos,
            std::bind(&BarcodeDecoder::subscription_callback, this, std::placeholders::_1));

    barcode_id_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/barcode_id", 10);

    barcode_raw_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/barcode_raw", 10);

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    RCLCPP_INFO(this->get_logger(), "码标解码器启动");
  }

private:
  void subscription_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg)
      return;

    // Frame skipping: only process every 3rd frame
    frame_count_++;
    if (frame_count_ % 3 != 0)
      return;

    int height = msg->height;
    int width = msg->width;
    size_t step = msg->step;

    if (height <= 0 || width <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "图像尺寸无效: 宽=%d, 高=%d", width, height);
      return;
    }

    if (step < static_cast<size_t>(width))
    {
      RCLCPP_WARN(this->get_logger(), "图像步长无效: step=%zu, width=%d", step, width);
      return;
    }

    if (msg->data.size() < step * static_cast<size_t>(height))
    {
      RCLCPP_WARN(this->get_logger(), "图像数据长度不足");
      return;
    }

    cv::Mat y_plane(height, width, CV_8UC1, msg->data.data(), step);

    cv::Mat gray;
    if (step == static_cast<size_t>(width) && y_plane.isContinuous())
    {
      gray = y_plane;
    }
    else
    {
      gray = y_plane.clone();
    }

    // Region-of-interest: only scan the center 60% of the image
    int roi_x = static_cast<int>(width * 0.2);
    int roi_y = static_cast<int>(height * 0.2);
    int roi_w = static_cast<int>(width * 0.6);
    int roi_h = static_cast<int>(height * 0.6);

    if (roi_w <= 0 || roi_h <= 0 || roi_x < 0 || roi_y < 0 ||
        roi_x + roi_w > width || roi_y + roi_h > height)
    {
      RCLCPP_WARN(this->get_logger(), "ROI区域越界，使用全幅");
      roi_x = 0;
      roi_y = 0;
      roi_w = width;
      roi_h = height;
    }

    cv::Mat roi_gray = gray(cv::Rect(roi_x, roi_y, roi_w, roi_h));

    zbar::Image zbar_image(
        roi_w,
        roi_h,
        "Y800",
        roi_gray.data,
        roi_w * roi_h);

    int result = scanner_.scan(zbar_image);

    if (result <= 0)
    {
      // No barcode found this frame, reset debounce
      debounce_counter_ = 0;
      last_barcode_id_ = 0;
      last_barcode_raw_ = "";
      return;
    }

    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end();
         ++symbol)
    {
      std::string barcode_data = symbol->get_data();

      std_msgs::msg::Int32 barcode_id_msg;
      std_msgs::msg::String barcode_raw_msg;

      bool valid_barcode = true;

      if (barcode_data == "ClockWise")
      {
        barcode_id_msg.data = 1;
      }
      else if (barcode_data == "AntiClockWise")
      {
        barcode_id_msg.data = 2;
      }
      else
      {
        try
        {
          int number = std::stoi(barcode_data);

          if (number >= 1 && number <= 9999)
          {
            // odd -> direction_id=1, even -> direction_id=2
            barcode_id_msg.data = (number % 2 == 0) ? 2 : 1;
          }
          else
          {
            RCLCPP_WARN(
                this->get_logger(),
                "识别数字超出范围 (1-9999): %d",
                number);

            valid_barcode = false;
          }
        }
        catch (const std::invalid_argument &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "无法识别的内容: %s",
              barcode_data.c_str());

          valid_barcode = false;
        }
        catch (const std::out_of_range &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "数字超出int范围: %s",
              barcode_data.c_str());

          valid_barcode = false;
        }
      }

      if (!valid_barcode)
      {
        debounce_counter_ = 0;
        last_barcode_id_ = 0;
        last_barcode_raw_ = "";
        continue;
      }

      // Debounce: require 2 consistent reads to avoid false positives
      if (barcode_id_msg.data == last_barcode_id_ && barcode_data == last_barcode_raw_)
      {
        debounce_counter_++;
        if (debounce_counter_ >= 2)
        {
          // Publish raw barcode result
          barcode_raw_msg.data = barcode_data;
          barcode_raw_publisher_->publish(barcode_raw_msg);

          // Publish translated direction ID
          barcode_id_publisher_->publish(barcode_id_msg);

          RCLCPP_INFO(
              this->get_logger(),
              "码标原始: %s, 方向结果: %d",
              barcode_data.c_str(),
              barcode_id_msg.data);

          debounce_counter_ = 0;
        }
      }
      else
      {
        last_barcode_id_ = barcode_id_msg.data;
        last_barcode_raw_ = barcode_data;
        debounce_counter_ = 1;
      }
    }
  }

private:
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscriber_hbmem_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr barcode_id_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_raw_publisher_;

  zbar::ImageScanner scanner_;

  int frame_count_;
  int debounce_counter_;
  int last_barcode_id_;
  std::string last_barcode_raw_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BarcodeDecoder>());
  rclcpp::shutdown();
  return 0;
}
