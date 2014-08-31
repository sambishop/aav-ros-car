cmake_minimum_required(VERSION 2.8)
project(cam_tracker)
find_package(OpenCV REQUIRED)
add_executable(calibrate calibrate.cpp)
add_executable(capture capture.cpp)
add_executable(manipulate manipulate.cpp)
add_executable(centroid centroid.cpp)
add_executable(cam_tracker
    cam_tracker.cpp

    VideoSource.h
    CameraVideoSource.h
    CameraVideoSource.cpp
    FileVideoSource.h
    FileVideoSource.cpp

    VideoSink.h
    FileVideoSink.h
    FileVideoSink.cpp
    NullVideoSink.h
    NullVideoSink.cpp

    ModeBuilder.h
    CaptureModeBuilder.h
    CaptureModeBuilder.cpp
    PlaybackModeBuilder.h
    PlaybackModeBuilder.cpp
)
target_link_libraries(calibrate ${OpenCV_LIBS})
target_link_libraries(capture ${OpenCV_LIBS})
target_link_libraries(manipulate ${OpenCV_LIBS})
target_link_libraries(centroid ${OpenCV_LIBS})
target_link_libraries(cam_tracker
    ${OpenCV_LIBS}
)

