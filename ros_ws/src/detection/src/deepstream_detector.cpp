/** \file detector.cpp
 * \brief Deepstream Detection node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "deepstream_detector.hpp"

// Std Includes

#include <functional>

// ROS Includes

#include "detection/msg/Detections.h"

// Deepstream includes

#include "deepstream_app.h"
#include "gst/gst.h"

namespace {
std::function<void(detection::msg::Detections&)> det_callback;
}

extern "C" {

/** \fn deepstream_app_main
 * \brief Forward declaration of the extern app
 */
extern int deepstream_app_main(int argc, const char** argv);

void deepstreamCallback(void* appCtx_v, void* batch_meta_v) {
    NvDsBatchMeta* batch_meta = (NvDsBatchMeta*)batch_meta_v;
    AppCtx* appCtx = (AppCtx*)appCtx_v;

    detection::msg::Detections dets;

    for (NvDsMetaList* l_frame = (NvDsMetaList*)batch_meta->frame_meta_list;
         l_frame != NULL; l_frame = l_frame->next) {
        NvDsFrameMeta* frame_meta = (NvDsFrameMeta*)l_frame->data;

        for (NvDsMetaList* l_obj = frame_meta->obj_meta_list; l_obj != NULL;
             l_obj = l_obj->next) {
            NvDsObjectMeta* obj = (NvDsObjectMeta*)l_obj->data;

            detection::msg::Detection det;

            det.clss = obj->class_id;
            det.score = obj->confidence;
            det.x = obj->rect_params.left;
            det.y = obj->rect_params.top;
            det.w = obj->rect_params.width;
            det.h = obj->rect_params.height;

            dets.detections.push_back(det);
        }
    }
    det_callback(dets);
}
}

DeepstreamDetector::DeepstreamDetector() {
    pub_detections = create_publisher<polystar_msgs::msg::Detections>()

    auto deepstream_config = get_parameter("net.deepstream").as_string("detections", 1);

    setupNet(deepstream_config);
}

void DeepstreamDetector::setupNet(const std::string& deepstream_config) {
    fake_argc = 3;
    fake_argv.push_back("deepstream-app");
    fake_argv.push_back("-c");
    fake_argv.push_back(const_cast<char*>(deepstream_config.c_str()));

    det_callback = [this](auto& d) { this->callback(d); };
}

void DeepstreamDetector::run() {
    deepstream_app_main(fake_argc, fake_argv.data());
}

void DeepstreamDetector::callback(polystar_msgs::msg::Detections& dets) {
    pub_detections->publish(dets);
}
