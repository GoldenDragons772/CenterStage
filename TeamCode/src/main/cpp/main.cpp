#include <jni.h>
#include <string>

#include <libusb.h>
#include "depthai/depthai.hpp"


extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_OakPrint(JNIEnv *env, jobject obj) {

//    auto r = libusb_set_option(nullptr, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
//
//     dai::Device device;
//
//     device = dai::Device(dai::OpenVINO::VERSION_2022_1, dai::UsbSpeed::HIGH);
//
//    dai::Pipeline pipeline;
//
//    device.startPipeline(pipeline);`

    std::string hello = "Cant Belive thsi Works";
    return env->NewStringUTF(hello.c_str());
}