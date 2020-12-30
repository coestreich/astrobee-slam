# cmake_minimum_required(VERSION 2.8.3)
project(test_session_tools)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_test.py DESTINATION bin PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE)

install_launch_files()