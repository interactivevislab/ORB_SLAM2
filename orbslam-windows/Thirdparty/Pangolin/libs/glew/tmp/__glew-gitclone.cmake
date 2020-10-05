
if(NOT "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitinfo.txt" IS_NEWER_THAN "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  clone --no-checkout "https://github.com/Perlmint/glew-cmake.git" "__glew"
    WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/Perlmint/glew-cmake.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"  checkout 7574ab4d00b683e56adbfdec7da636529dfe65d8 --
  WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '7574ab4d00b683e56adbfdec7da636529dfe65d8'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  submodule update --recursive --init 
    WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitinfo.txt"
    "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/glew/src/__glew-stamp/__glew-gitclone-lastrun.txt'")
endif()

