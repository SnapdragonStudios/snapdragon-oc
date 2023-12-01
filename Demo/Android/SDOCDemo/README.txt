Android Project Setup

This could be thought as mini-integration of SDOC
1. Developers could replace /sdcard/Android/data/com.qualcomm.sdocdemo/files/input.cap to check new capture's result
2. The output image is /sdcard/Android/data/com.qualcomm.sdocdemo/files/depthbuffer.pgm
3. Developers could check the log as it covers most of important apis

Usage for developers:
1. Quickly know the status of SDOC on available platforms
2. Validate the result of captures, for example, developers could capture 1000 captures and run the demo for validation

Notices:
Refer to app/build.gradle, currently the build target abiFilters is set to armv7

To make sure .so file can be correctly put in generated apk, developers should remove app/build folder before building apk.