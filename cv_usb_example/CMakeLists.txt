project (cv_capture_example)
cmake_minimum_required(VERSION 2.8)

# CONFIGURE OPENCV
find_package (OpenCV REQUIRED)

# CREATE EXECUTABLE
add_executable (cv_capture_example cv_capture_example.cpp)
target_link_libraries(cv_capture_example ${OpenCV_LIBS})
