# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" rev-list --max-count=1 HEAD
  WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
  RESULT_VARIABLE error_code
  OUTPUT_VARIABLE head_sha
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )
if(error_code)
  message(FATAL_ERROR "Failed to get the hash for HEAD")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" show-ref "bc8f8be222287fec977ec3f47a5cb065cceb2ee9"
  WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
  OUTPUT_VARIABLE show_ref_output
  )
# If a remote ref is asked for, which can possibly move around,
# we must always do a fetch and checkout.
if("${show_ref_output}" MATCHES "remotes")
  set(is_remote_ref 1)
else()
  set(is_remote_ref 0)
endif()

# Tag is in the form <remote>/<tag> (i.e. origin/master) we must strip
# the remote from the tag.
if("${show_ref_output}" MATCHES "refs/remotes/bc8f8be222287fec977ec3f47a5cb065cceb2ee9")
  string(REGEX MATCH "^([^/]+)/(.+)$" _unused "bc8f8be222287fec977ec3f47a5cb065cceb2ee9")
  set(git_remote "${CMAKE_MATCH_1}")
  set(git_tag "${CMAKE_MATCH_2}")
else()
  set(git_remote "origin")
  set(git_tag "bc8f8be222287fec977ec3f47a5cb065cceb2ee9")
endif()

# This will fail if the tag does not exist (it probably has not been fetched
# yet).
execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" rev-list --max-count=1 "${git_tag}"
  WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
  RESULT_VARIABLE error_code
  OUTPUT_VARIABLE tag_sha
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

# Is the hash checkout out that we want?
if(error_code OR is_remote_ref OR NOT ("${tag_sha}" STREQUAL "${head_sha}"))
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" fetch
    WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
    RESULT_VARIABLE error_code
    )
  if(error_code)
    message(FATAL_ERROR "Failed to fetch repository 'https://github.com/LuaDist/libjpeg.git'")
  endif()

  if(is_remote_ref)
    # Check if stash is needed
    execute_process(
      COMMAND "C:/Program Files/Git/cmd/git.exe" status --porcelain
      WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
      RESULT_VARIABLE error_code
      OUTPUT_VARIABLE repo_status
      )
    if(error_code)
      message(FATAL_ERROR "Failed to get the status")
    endif()
    string(LENGTH "${repo_status}" need_stash)

    # If not in clean state, stash changes in order to be able to perform a
    # rebase or checkout without losing those changes permanently
    if(need_stash)
      execute_process(
        COMMAND "C:/Program Files/Git/cmd/git.exe" stash save --all;--quiet
        WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
        RESULT_VARIABLE error_code
        )
      if(error_code)
        message(FATAL_ERROR "Failed to stash changes")
      endif()
    endif()

    if("REBASE" STREQUAL "CHECKOUT")
      execute_process(
        COMMAND "C:/Program Files/Git/cmd/git.exe" checkout "${git_remote}/${git_tag}"
        WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
        RESULT_VARIABLE error_code
        )
      if(error_code)
        message(FATAL_ERROR "Failed to checkout tag: '${git_remote}/${git_tag}'")
      endif()
    else()
      # Pull changes from the remote branch
      execute_process(
        COMMAND "C:/Program Files/Git/cmd/git.exe" rebase "${git_remote}/${git_tag}"
        WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
        RESULT_VARIABLE error_code
        OUTPUT_VARIABLE rebase_output
        ERROR_VARIABLE  rebase_output
        )
      if(error_code)
        # Rebase failed, undo the rebase attempt before continuing
        execute_process(
          COMMAND "C:/Program Files/Git/cmd/git.exe" rebase --abort
          WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
        )

        if(NOT "REBASE" STREQUAL "REBASE_CHECKOUT")
          # Not allowed to do a checkout as a fallback, so cannot proceed
          if(need_stash)
            execute_process(
              COMMAND "C:/Program Files/Git/cmd/git.exe" stash pop --index --quiet
              WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
              )
          endif()
          message(FATAL_ERROR "\nFailed to rebase in: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg'."
                              "\nOutput from the attempted rebase follows:"
                              "\n${rebase_output}"
                              "\n\nYou will have to resolve the conflicts manually")
        endif()

        # Fall back to checkout. We create an annotated tag so that the user
        # can manually inspect the situation and revert if required.
        # We can't log the failed rebase output because MSVC sees it and
        # intervenes, causing the build to fail even though it completes.
        # Write it to a file instead.
        string(TIMESTAMP tag_timestamp "%Y%m%dT%H%M%S" UTC)
        set(tag_name _cmake_ExternalProject_moved_from_here_${tag_timestamp}Z)
        set(error_log_file ${CMAKE_CURRENT_LIST_DIR}/rebase_error_${tag_timestamp}Z.log)
        file(WRITE ${error_log_file} "${rebase_output}")
        message(WARNING "Rebase failed, output has been saved to ${error_log_file}"
                        "\nFalling back to checkout, previous commit tagged as ${tag_name}")
        execute_process(
          COMMAND "C:/Program Files/Git/cmd/git.exe" tag -a
                  -m "ExternalProject attempting to move from here to ${git_remote}/${git_tag}"
                  ${tag_name}
          WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          RESULT_VARIABLE error_code
        )
        if(error_code)
          message(FATAL_ERROR "Failed to add marker tag")
        endif()

        execute_process(
          COMMAND "C:/Program Files/Git/cmd/git.exe" checkout "${git_remote}/${git_tag}"
          WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          RESULT_VARIABLE error_code
        )
        if(error_code)
          message(FATAL_ERROR "Failed to checkout : '${git_remote}/${git_tag}'")
        endif()

      endif()
    endif()

    if(need_stash)
      execute_process(
        COMMAND "C:/Program Files/Git/cmd/git.exe" stash pop --index --quiet
        WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
        RESULT_VARIABLE error_code
        )
      if(error_code)
        # Stash pop --index failed: Try again dropping the index
        execute_process(
          COMMAND "C:/Program Files/Git/cmd/git.exe" reset --hard --quiet
          WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          RESULT_VARIABLE error_code
          )
        execute_process(
          COMMAND "C:/Program Files/Git/cmd/git.exe" stash pop --quiet
          WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          RESULT_VARIABLE error_code
          )
        if(error_code)
          # Stash pop failed: Restore previous state.
          execute_process(
            COMMAND "C:/Program Files/Git/cmd/git.exe" reset --hard --quiet ${head_sha}
            WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          )
          execute_process(
            COMMAND "C:/Program Files/Git/cmd/git.exe" stash pop --index --quiet
            WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
          )
          message(FATAL_ERROR "\nFailed to unstash changes in: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg'."
                              "\nYou will have to resolve the conflicts manually")
        endif()
      endif()
    endif()
  else()
    execute_process(
      COMMAND "C:/Program Files/Git/cmd/git.exe" checkout "${git_tag}"
      WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
      RESULT_VARIABLE error_code
      )
    if(error_code)
      message(FATAL_ERROR "Failed to checkout tag: '${git_tag}'")
    endif()
  endif()

  set(init_submodules "TRUE")
  if(init_submodules)
    execute_process(
      COMMAND "C:/Program Files/Git/cmd/git.exe" submodule update --recursive --init 
      WORKING_DIRECTORY "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg"
      RESULT_VARIABLE error_code
      )
  endif()
  if(error_code)
    message(FATAL_ERROR "Failed to update submodules in: 'F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/libjpeg/src/__libjpeg'")
  endif()
endif()
