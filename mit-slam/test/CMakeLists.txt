add_executable(test_BlobTracker test_BlobTracker.cxx)
target_link_libraries(test_BlobTracker
  LINK_PUBLIC ${GTEST_LIBRARIES} mit-slam)
target_include_directories(test_BlobTracker PUBLIC ${GTEST_INCLUDE_DIRS})

add_test(NAME test_BlobTracker
  COMMAND test_BlobTracker
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)


add_executable(test_MsgsUtils test_MsgsUtils.cxx)
target_link_libraries(test_MsgsUtils
  LINK_PUBLIC ${GTEST_LIBRARIES} mit-slam)
target_include_directories(test_MsgsUtils PUBLIC ${GTEST_INCLUDE_DIRS})

add_test(NAME test_MsgsUtils
  COMMAND test_MsgsUtils
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)
