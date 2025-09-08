/** \file detector.cpp
 * \brief Detection node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "detector.hpp"

// Std includes

#include <fstream>
#include <vector>
#include <chrono>

// Darknet

// Stupid namespace clash between Darknet and ROS messages, so this quick hack
// allows the file to compile. Since darknet is written in C and no mangling is
// done, this doesn't cause linker issues (so far!)
#define detection detection_darknet
#include "darknet.h"
#include "parser.h"
#undef detection

// ROS Includes

#include "polystar_msgs/msg/detections.hpp"
#include "cv_bridge/cv_bridge.hpp"

namespace {
// Darknet interface functions

/** \fn findBestClass
 * \brief Returns the best class index from an array of scores
 */
int findBestClass(float* probabilities, int nclass, float treshold) {
    float curr_max = 0.f;
    int curr_idx = -1;
    for (auto i = 0u; i < nclass; ++i) {
        if (probabilities[i] > curr_max && probabilities[i] >= treshold) {
            curr_max = probabilities[i];
            curr_idx = i;
        }
    }
    return curr_idx;
}
} // namespace

struct Detector::impl {
    // TODO : fetch from parameter server
    float nms = 0.45f;
    float tresh = 0.24f;
    float hier_tresh = 0.5f;

    std::vector<std::string> labels;
    network net;
};

Detector::Detector() : Node("detection"), p(std::make_unique<impl>()) {
    auto datacfg = get_parameter("net.datacfg").as_string();
    auto config_path = get_parameter("net.config_path").as_string();
    auto weights_path = get_parameter("net.weights").as_string();

    setupNet(datacfg, config_path, weights_path);

    pub_detections_ = create_publisher<polystar_msgs::msg::Detections>("detections", 1);

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        "image_in", 1, [this](const std::shared_ptr<const sensor_msgs::msg::Image>& msg){
        imageCallback(msg);
    });
}

Detector::~Detector() = default;

void Detector::setupNet(const std::string& datacfg,
                        const std::string& config_path,
                        const std::string& weights_path) {
    loadLabels();

    // Load network
    // const_casts because the API is pretty badly designed -_-
    p->net =
        parse_network_cfg_custom(const_cast<char*>(config_path.c_str()), 1, 1);
    load_weights(&p->net, const_cast<char*>(weights_path.c_str()));
    fuse_conv_batchnorm(p->net);
    calculate_binary_weights(p->net);

    if (p->net.layers[p->net.n - 1].classes != p->labels.size()) {
        throw std::runtime_error(
            "Different number of classes between network and label file");
    }

    std::cout << "Detector : Ready\n";
}

void Detector::loadLabels() {
    std::string path;
    if (!this->get_parameter("net.labels", path)) {
        return;
    }

    std::cout << "Loading labels : \n";

    std::ifstream labels_file(path);

    std::string buffer;
    while (std::getline(labels_file, buffer)) {
        p->labels.emplace_back(buffer);
        std::cout << '\t' << buffer << '\n';
        buffer = "";
    }
}

image matToImage(cv::Mat& mat) {
    // From darknet/src/image_opencv.cpp
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();
    image im = make_image(w, h, c);
    unsigned char* data = (unsigned char*)mat.data;
    int step = mat.step;
    for (int y = 0; y < h; ++y) {
        for (int k = 0; k < c; ++k) {
            for (int x = 0; x < w; ++x) {

                im.data[k * w * h + y * w + x] =
                    data[y * step + x * c + k] / 255.0f;
            }
        }
    }
    return im;
}

void Detector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    constexpr float ratio = 1.f / 256.f;

    std::cout << "Incoming frame : " << img->header.frame_id << '\n';

    auto img_opencv = cv_bridge::toCvCopy(img);

    // TODO : check conversion
    // img_opencv->image.convertTo(img_float, CV_32F, ratio);

    image im = matToImage((*img_opencv).image);

    image sized = resize_image(im, p->net.w, p->net.h);

    auto t0 = std::chrono::steady_clock::now();
    // Run inference
    network_predict(p->net, sized.data);
    auto t1 = std::chrono::steady_clock::now();


    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << '\n';
    // Fetch boxes
    int nboxes = 0;
    detection_darknet* dets = get_network_boxes(
        &p->net, im.w, im.h, p->tresh, p->hier_tresh, 0, 1, &nboxes, 0);

    polystar_msgs::msg::Detections msg;
    // Construct ROS message
    for (auto i = 0; i < nboxes; ++i) {
        auto& d = dets[i];

        for (auto j = 0; j < p->labels.size(); ++j) {
            std::cout << d.prob[j] << ' ';
        }

        d.best_class_idx = findBestClass(d.prob, p->labels.size(), p->tresh);

        if (d.best_class_idx == -1) {
            // No best match found : continue handling rest of detection
            continue;
        }

        std::cout << ", best " << p->labels[d.best_class_idx] << '\n';

        polystar_msgs::msg::Detection det;
        det.x = d.bbox.x * im.w;
        det.y = d.bbox.y * im.h;
        det.w = d.bbox.w * im.w;
        det.h = d.bbox.h * im.h;
        det.clss = d.best_class_idx;
        det.score = d.prob[det.clss];

        msg.detections.push_back(det);
    }

    std::cout << '\n';

    pub_detections_->publish(msg);

    free_detections(dets, nboxes);
    free_image(sized);
    free_image(im);
}
