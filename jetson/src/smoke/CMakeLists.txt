cmake_minimum_required(VERSION 3.10)
project(smoke)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(smoke test/test_smoke.py TIMEOUT 30)
endif()

ament_package()
