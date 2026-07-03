#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "web_video_server::web_video_server" for configuration "Release"
set_property(TARGET web_video_server::web_video_server APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(web_video_server::web_video_server PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libweb_video_server.so"
  IMPORTED_SONAME_RELEASE "libweb_video_server.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS web_video_server::web_video_server )
list(APPEND _IMPORT_CHECK_FILES_FOR_web_video_server::web_video_server "${_IMPORT_PREFIX}/lib/libweb_video_server.so" )

# Import target "web_video_server::web_video_server_streamers" for configuration "Release"
set_property(TARGET web_video_server::web_video_server_streamers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(web_video_server::web_video_server_streamers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libweb_video_server_streamers.so"
  IMPORTED_SONAME_RELEASE "libweb_video_server_streamers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS web_video_server::web_video_server_streamers )
list(APPEND _IMPORT_CHECK_FILES_FOR_web_video_server::web_video_server_streamers "${_IMPORT_PREFIX}/lib/libweb_video_server_streamers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
