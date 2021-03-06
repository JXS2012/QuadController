cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(VectorComputation src/VectorComputation)

rosbuild_add_library(Potential src/Potential.cpp)
target_link_libraries(Potential VectorComputation)

rosbuild_add_library(NewPotential src/NewPotential.cpp)
target_link_libraries(NewPotential VectorComputation)

#rosbuild_add_executable(PotentialTest src/PotentialTest.cpp)
#target_link_libraries(PotentialTest Potential)

rosbuild_add_library(DummyBird src/DummyBird.cpp)
target_link_libraries(DummyBird VectorComputation)

rosbuild_add_library(BirdEye src/BirdEye.cpp)
target_link_libraries(BirdEye VectorComputation)

rosbuild_add_library(SonarEye src/SonarEye.cpp)
target_link_libraries(SonarEye VectorComputation)

rosbuild_add_library(HighBird src/HighBird.cpp)
target_link_libraries(HighBird DummyBird)
target_link_libraries(HighBird BirdEye)
#target_link_libraries(HighBird SonarEye)
target_link_libraries(HighBird Potential)

rosbuild_add_library(VisualBird src/VisualBird.cpp)
target_link_libraries(VisualBird DummyBird)
target_link_libraries(VisualBird BirdEye)
#target_link_libraries(VisualBird SonarEye)
target_link_libraries(VisualBird Potential)

rosbuild_add_library(VisualBirdVer5 src/VisualBirdVer5.cpp)
target_link_libraries(VisualBirdVer5 DummyBird)
target_link_libraries(VisualBirdVer5 BirdEye)
#target_link_libraries(VisualBird SonarEye)
target_link_libraries(VisualBirdVer5 NewPotential)

rosbuild_add_library(SonarBird src/SonarBird.cpp)
target_link_libraries(SonarBird DummyBird)
#target_link_libraries(SonarBird BirdEye)
target_link_libraries(SonarBird SonarEye)

rosbuild_add_executable(quadSmart src/quadSmart.cpp)
target_link_libraries(quadSmart HighBird)

rosbuild_add_executable(quadSonar src/quadSonar.cpp)
target_link_libraries(quadSonar SonarBird)

rosbuild_add_executable(quadVisual src/quadVisual.cpp)
target_link_libraries(quadVisual VisualBird)

rosbuild_add_executable(quadVirtual src/quadVirtual.cpp)
target_link_libraries(quadVirtual VisualBirdVer5)


rosbuild_add_executable(DummbyTest src/DummyTest.cpp)
target_link_libraries(DummbyTest DummyBird)


rosbuild_add_executable(viconTranslator src/viconTranslator.cpp)
#rosbuild_add_executable(quadView src/quadView.cpp)
#target_link_libraries(quadView BirdEye)



rosbuild_add_executable(testControl src/testControl.cpp)
rosbuild_add_executable(angleTransform src/angleTransform.cpp)
#rosbuild_add_executable(speedtest src/speedtest.cpp)
#rosbuild_add_executable(targetObserver src/targetObserver.cpp)

rosbuild_add_executable(targetObserver src/targetObserverServer.cpp)

#rosbuild_add_library(smartbird src/smartbird.cpp)
#target_link_libraries(smartbird dummybird)
#rosbuild_add_executable(quadFlight src/quadFlight.cpp)
#target_link_libraries(quadFlight quadrotor)
#rosbuild_add_executable(quadShift src/quadShift.cpp)
#rosbuild_add_executable(quadController src/quadController.cpp)
#rosbuild_add_executable(tfListen src/tfListen.cpp)
