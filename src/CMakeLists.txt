add_subdirectory(vsnakes)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
if(BUILD_APPS)
  add_subdirectory(apps)
endif()

