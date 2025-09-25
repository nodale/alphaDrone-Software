include(FetchContent)
FetchContent_Declare(
  openvins  
  GIT_REPOSITORY https://github.com/rpng/open_vins/ 
  GIT_TAG 93adc241390d13e99232652cf05cbe18a93c7bea)
FetchContent_MakeAvailable(openvins)
