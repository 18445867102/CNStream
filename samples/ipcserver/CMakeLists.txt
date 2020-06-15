include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/settings.cmake)
include_directories("${Example_INCLUDE_DIRS}")
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} srcs)
set(Server_src server_demo.cpp)
get_filename_component(name_server "${Server_src}" NAME_WE)
message("target :  ${name}")

if(build_rtsp_sink)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/live555/lib)
endif()

add_executable(${name_server} ${srcs} ${preprocess_srcs} ${postprocess_srcs} ${obj_filter_srcs})

if (SANITIZE_MEMORY OR SANITIZE_ADDRESS OR SANITIZE_THREAD OR SANITIZE_UNDEFINED)
    add_sanitizers(${name_server})
endif()

target_link_libraries(${name_server} ${Example_LINKER_LIBS} rt)

if(build_modules_contrib)
  #force load all the modules
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
  target_link_libraries(${name_server} cnstream_contrib)
endif()
