cmake_minimum_required(VERSION 2.8)

set (CMAKE_CXX_STANDARD 11)

set(OPENCV_INCLUDE_PATH "/home/anirudh/SDK/open_cv3_3_1_suraj/include")
set(OPENCV_LIBRARIES_PATH "/home/anirudh/SDK/open_cv3_3_1_suraj/lib")
set(OPENCV_LINK_LIBRARIES "-lopencv_core -lopencv_imgproc -lopencv_objdetect -lopencv_features2d -lopencv_calib3d -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_bioinspired")


set(SYS_INCLUDE_PATH "/usr/include/linux-x86_64")
set(SYS_LIBRARIES_PATH "/usr/lib/x86_64-linux-gnu")

set(AR_INCLUDE_PATH "/home/anirudh/SDK/Ar_toolkit/include")
set(AR_LIBRARIES_PATH "/home/anirudh/SDK/Ar_toolkit/lib")
set(AR_LINK_LIBRARIES "-lKPM -lAR2 -lARUtil -lARgsub_lite -lARvideo -lAR -lARICP -lAR -lglut -lGLU -lGL -lX11 -lm -lpthread -ljpeg -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -pthread -lgthread-2.0 -pthread -lglib-2.0 -lxml2")


include_directories(${OPENCV_INCLUDE_PATH})
include_directories(${AR_INCLUDE_PATH})

link_directories(${OPENCV_LIBRARIES_PATH})
link_directories(${AR_LIBRARIES_PATH})

file(GLOB IPS_PROJECT_SOURCE_FILES "*.h" "*.cpp" "*.c")

project(opencv_ar)
add_executable(${PROJECT_NAME} ${IPS_PROJECT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LINK_LIBRARIES} ${AR_LINK_LIBRARIES} -lpthread -ldl)
