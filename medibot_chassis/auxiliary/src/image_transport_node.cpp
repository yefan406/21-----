// Copyright (c) 2022, www.guyuehome.com

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

using hbm_img_msgs::msg::HbmMsg1080P;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class MedibotNv122BGR : public rclcpp::Node
{
public:
    MedibotNv122BGR(std::string node_name = "medibot_nv12_to_bgr") : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), node_name.c_str());

        // Create message subscriber from camera node
        sublisher_ = this->create_subscription<HbmMsg1080P>(
            "/hbmem_img", 10, std::bind(&MedibotNv122BGR::image_callback, this, std::placeholders::_1));
        compress_pub_ = this->create_publisher<CompressedImage>("/medibot_image_out/compressed", 10);
        image_pub_ = this->create_publisher<Image>("/medibot_image_out", 10);
    }

private:
    void image_callback(const HbmMsg1080P::ConstSharedPtr img_msg)
    {
        // Validate incoming image, only NV12 format supported
        if (!img_msg)
            return;
        if ("nv12" != std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
        {
            RCLCPP_ERROR(rclcpp::get_logger("medibot_nv12_to_bgr"), "Only NV12 image encoding supported");
            return;
        }
        cv::Mat nv12Image(img_msg->height * 3 / 2, img_msg->width, CV_8UC1,
                          const_cast<uint8_t *>(img_msg->data.data()));
        cv::Mat bgrImage;
        cv::cvtColor(nv12Image, bgrImage, cv::COLOR_YUV2BGR_NV12);

        auto cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgrImage);
        compress_pub_->publish(std::move(*cvImage.toCompressedImageMsg(cv_bridge::JPEG)));
        image_pub_->publish(std::move(*cvImage.toImageMsg()));
    }
    rclcpp::Publisher<CompressedImage>::SharedPtr compress_pub_;
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    rclcpp::Subscription<HbmMsg1080P>::ConstSharedPtr sublisher_ = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MedibotNv122BGR>("medibot_nv12_to_bgr");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
