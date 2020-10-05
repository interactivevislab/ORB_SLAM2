
if(NOT "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitinfo.txt" IS_NEWER_THAN "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  clone --no-checkout "https://github.com/LuaDist/libjpeg.git" "__libjpeg"
    WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/LuaDist/libjpeg.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"  checkout bc8f8be222287fec977ec3f47a5cb065cceb2ee9 --
  WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'bc8f8be222287fec977ec3f47a5cb065cceb2ee9'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  submodule update --recursive --init 
    WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitinfo.txt"
    "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg-stamp/__libjpeg-gitclone-lastrun.txt'")
endif()

