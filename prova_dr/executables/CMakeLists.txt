link_libraries(${OpenCV_LIBS})

#test mapping
add_executable(test_mapping test_mapping.cpp
../src/camera.cpp
../src/image.cpp
../src/camera.cpp
#../src/renderer.cpp
../src/environment.cpp
../src/dtam.cpp
../src/utils.cpp
../src/mapper.cpp
../src/tracker.cpp
)
