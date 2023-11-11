#include <jni.h>
#include <string>

#include <libusb.h>
#include "depthai/depthai.hpp"


std::shared_ptr<dai::Device> device;
static std::atomic<bool> syncNN{true};

extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_OakPrint(JNIEnv *env, jobject obj) {
    std::string hello = "Cant Belive thsi Works";
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_startDevice(JNIEnv *env, jobject obj, int width, int height) {

    auto r = libusb_set_option(nullptr, LIBUSB_OPTION_ANDROID_JNIENV, env);

    device = std::make_shared<dai::Device>(dai::OpenVINO::VERSION_2022_1, dai::UsbSpeed::HIGH);

    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    camRgb->setPreviewSize(width, height);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    camRgb->preview.link(xoutRgb->input);

    device->startPipeline(pipeline);
}