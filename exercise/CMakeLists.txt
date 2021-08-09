project(yuv_pic)
find_package(OpenCV)
add_executable(yuv_pic
  yuv_pic.cpp
)
target_link_libraries(yuv_pic
  ${OpenCV_LIBS}
)
