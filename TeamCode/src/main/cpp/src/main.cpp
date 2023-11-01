#include <jni.h>

extern "C"
JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_Oak_CameraPreview_OakPrint(JNIEnv *env, jobject obj) {
    return env->NewStringUTF("Hello World!");
}