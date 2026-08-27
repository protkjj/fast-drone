# fast_drone 공력 플러그인 + 검증 테스트
#
# 두 타깃이 있고, 의존성이 다르다:
#   test_aero_core  — 검증. gz 가 **없어도** 빌드된다 (macOS 개발용)
#   FastDroneAero   — Gazebo 플러그인 (.so). gz-sim8 이 있을 때만 빌드
#
# 이렇게 나눈 이유는 DESIGN.md 4장이다. 수식 검증은 Gazebo 와 무관해야 하고,
# 무관해야 macOS 에서 고치고 Ubuntu 에서 확인하는 왕복이 짧아진다.
#
#   mkdir -p build && cd build && cmake .. && make -j
#   ctest --output-on-failure          (검증 실행)
cmake_minimum_required(VERSION 3.16)
project(fast_drone_gz_aero CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# ── 1) 검증 테스트 (gz 불필요) ────────────────────────────────────────────
add_executable(test_aero_core test/test_aero_core.cc)
target_include_directories(test_aero_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

enable_testing()
foreach(src sized placeholder)
  add_test(NAME aero_core_${src}
           COMMAND test_aero_core
                   ${CMAKE_CURRENT_SOURCE_DIR}/data/aero_${src}.csv
                   ${CMAKE_CURRENT_SOURCE_DIR}/test/aero_reference_${src}.csv
                   ${CMAKE_CURRENT_SOURCE_DIR}/test/aero_interp_${src}.csv)
endforeach()

# ── 2) Gazebo 플러그인 (gz-sim8 = Harmonic 이 있을 때만) ──────────────────
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET COMPONENTS register)

if(gz-sim8_FOUND AND gz-plugin2_FOUND)
  message(STATUS "gz-sim8 발견 -> 플러그인 FastDroneAero 를 빌드합니다")
  add_library(FastDroneAero SHARED src/AeroPlugin.cc)
  target_include_directories(FastDroneAero
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(FastDroneAero
    PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
  # gz 는 접두사 없는 파일명(libFastDroneAero.so)을 그대로 찾는다
  set_target_properties(FastDroneAero PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  install(TARGETS FastDroneAero LIBRARY DESTINATION lib)
else()
  message(STATUS "gz-sim8 없음 -> 검증 테스트만 빌드합니다 "
                 "(macOS 개발 환경이면 정상)")
endif()
