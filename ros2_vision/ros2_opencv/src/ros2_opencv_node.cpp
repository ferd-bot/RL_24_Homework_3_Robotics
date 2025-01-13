#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BlobDetectorNode : public rclcpp::Node {
public:
    BlobDetectorNode() : Node("blob_detector_node") {
        // Subscriber per il flusso di immagini
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/videocamera", 10,
            std::bind(&BlobDetectorNode::image_callback, this, std::placeholders::_1));

        // Publisher per pubblicare l'immagine elaborata
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

        // Configurazione del rilevatore di blob
        cv::SimpleBlobDetector::Params params;
        params.filterByColor = false;
        params.filterByArea = true;
        params.minArea = 50;   // Area minima
        params.maxArea = 50000;  // Area massima
        params.filterByCircularity = true;
        params.minCircularity = 0.7; 
        params.filterByConvexity = false;
        params.filterByInertia = false;

        // Creazione del rilevatore
        detector_ = cv::SimpleBlobDetector::create(params);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Converti l'immagine ROS in OpenCV
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Converti l'immagine a scala di grigi
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

            // Rileva i blob
            std::vector<cv::KeyPoint> keypoints;
            detector_->detect(gray_image, keypoints);

            // Disegna i blob rilevati
            cv::Mat output_image;
            cv::drawKeypoints(image, keypoints, output_image, cv::Scalar(0, 0, 255),
                              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            // Verifica se sono stati rilevati blob
            if (!keypoints.empty()) {
                RCLCPP_INFO(this->get_logger(), "Sfera rilevata!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Nessun blob rilevato.");
            }

            // Converti l'immagine elaborata e pubblicala
            auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output_image).toImageMsg();
            image_publisher_->publish(*processed_msg);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Errore durante l'elaborazione dell'immagine: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    cv::Ptr<cv::SimpleBlobDetector> detector_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlobDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

