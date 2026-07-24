cmake_minimum_required(VERSION 3.10)

# project(mex_modules)

set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
# set(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
# set(CMAKE_INSTALL_LIBDIR ${ARCHIVE_OUTPUT_DIRECTORY})

set(PICADRIVE_ENABLE_LOGGER true)

add_subdirectory(third)
add_subdirectory(pica-drive)

find_package( Matlab COMPONENTS MX_LIBRARY )

if (Matlab_FOUND)
    # 配置正确，开始编译流程
    message(STATUS "MATLAB root directory found: ${Matlab_ROOT_DIR}.")

    matlab_add_mex(
        NAME mex_picadrive_bldc_foc
        MODULE
        mex_bldc_foc.cc
        LINK_TO ${ARCHIVE_OUTPUT_DIRECTORY}/libpicadrive.a
        LINK_TO spdlog::spdlog_header_only
    )

    set_target_properties(mex_picadrive_bldc_foc
        PROPERTIES
        COMPILE_FLAGS "-fvisibility=default"
        LIBRARY_OUTPUT_DIRECTORY ${MEX_OUTPUT_DIRECTORY} # mex 文件输出目录
    )

    target_include_directories(mex_picadrive_bldc_foc
        PRIVATE
            pica-drive/include
            pica-drive/src
            pica-drive/ports
            pica-drive/third/matrix
    )

    matlab_add_mex(
        NAME mex_picadrive_bldc_speed_pi
        MODULE
        mex_bldc_speed_pi.cc
        LINK_TO ${ARCHIVE_OUTPUT_DIRECTORY}/libpicadrive.a
        LINK_TO spdlog::spdlog_header_only
    )

    set_target_properties(mex_picadrive_bldc_speed_pi
        PROPERTIES
        COMPILE_FLAGS "-fvisibility=default"
        LIBRARY_OUTPUT_DIRECTORY ${MEX_OUTPUT_DIRECTORY} # mex 文件输出目录
    )

    target_include_directories(mex_picadrive_bldc_speed_pi
        PRIVATE
            pica-drive/include
            pica-drive/src
            pica-drive/ports
            pica-drive/third/matrix
    )

    message(STATUS "Mex file can be found in ${CMAKE_SOURCE_DIR}/bin")

else ()
    # 配置错误
    message(WARNING "MATLAB root directory not found. Failed to build MATLAB bindings.")
    set(BUILD_MATLAB_BINDINGS OFF)
endif ()
