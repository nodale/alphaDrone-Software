include(FetchContent)
FetchContent_Declare(
  nvblox  
  GIT_REPOSITORY https://github.com/nvidia-isaac/nvblox.git
  GIT_TAG b05ddea52d6dfc425d90d20b090eaa8ba6f12994)
FetchContent_MakeAvailable(nvblox)
