include(FetchContent)
FetchContent_Declare(
  realsense2
  GIT_REPOSITORY https://github.com/IntelRealSense/librealsense.git
  GIT_TAG 38a41441971387197193ad3aeae3cefe6a11f2cb)
FetchContent_MakeAvailable(realsense2)
