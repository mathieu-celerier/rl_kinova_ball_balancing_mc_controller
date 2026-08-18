cmake_minimum_required(VERSION 3.22)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME RLKinovaBallBalancingMcController)
set(PROJECT_DESCRIPTION "RLKinovaBallBalancingMcController")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()

# RLPolicyInterface::loadPolicy passes policy_path straight to std::filesystem / Ort::Session with
# no directory joining, so a bare filename resolves against the *current working directory* — i.e.
# it only works when mc_rtc is launched from inside policy/. Install the policies and bake their
# absolute location into the generated config instead.
set(RL_POLICY_INSTALL_DIR
    "${CMAKE_INSTALL_PREFIX}/share/RLKinovaBallBalancingMcController/policy")
install(
  DIRECTORY policy/
  DESTINATION share/RLKinovaBallBalancingMcController/policy
  FILES_MATCHING
  PATTERN "*.onnx")

configure_file(etc/RLKinovaBallBalancingMcController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/RLKinovaBallBalancingMcController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RLKinovaBallBalancingMcController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
