project (cv_usb_example)
cmake_minimum_required(VERSION 2.8)

find_package (OpenCV REQUIRED)

add_executable (cv_usb_example cv_usb_example.cpp)
target_link_libraries(cv_usb_example ${OpenCV_LIBS})
