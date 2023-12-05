#include <jni.h>
#include <string>
#include <iostream>

#include "depthai/depthai.hpp"
#include "opencv2/opencv.hpp"
#include <libusb.h>
#include <android/log.h>

#define LOG_TAG "depthaiAndroid"
#define log(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG, __VA_ARGS__)

std::shared_ptr<dai::Device> device;
std::string AprilTagCode;

extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_OakPrint(JNIEnv *env, jobject obj) {
    std::string hello = AprilTagCode;
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_startDevice(JNIEnv *env, jobject) {

    auto r = libusb_set_option(nullptr, LIBUSB_OPTION_ANDROID_JNIENV, env);
    log("libusb_set_option ANDROID_JAVAVM: %s", libusb_strerror(r));

    std::string ret;
    dai::Pipeline pipeline;

    try {
        dai::BoardConfig config;
        config.usb.pid = 0xf63b;
        pipeline.setBoardConfig(config);

        auto deviceInfoVec = dai::Device::getAllAvailableDevices();
        dai::Device device(pipeline, deviceInfoVec[0], dai::UsbSpeed::SUPER);

        //device.startPipeline(pipeline);
    } catch (const std::exception& e) {
        log("Oak-1 Failed: %s", e.what());
        ret = e.what();
    }
    return env->NewStringUTF(ret.c_str());
}