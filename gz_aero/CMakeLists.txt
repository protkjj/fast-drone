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

  # 힘 화살표(마커)는 있으면 켜고 없으면 끈다. 버전이 안 맞아 여기서 실패해도
  # 플러그인 본체는 그대로 빌드되어야 한다 — 시각화 때문에 공력을 잃을 순 없다.
  find_package(gz-transport13 QUIET)
  find_package(gz-msgs10 QUIET)
  if(gz-transport13_FOUND AND gz-msgs10_FOUND)
    target_compile_definitions(FastDroneAero PRIVATE FAST_DRONE_MARKERS=1)
    # 타깃 이름이 배포판마다 다르다(gz-transport13::gz-transport13 / ::core).
    # 틀린 이름을 그냥 적으면 CMake 가 죽어 공력까지 못 쓰게 되므로,
    # **있는 것만** 붙인다. gz-sim8 이 이미 전이 링크하므로 하나도 안 붙어도
    # 대개 빌드된다.
    foreach(t gz-transport13::gz-transport13 gz-transport13::core
              gz-msgs10::gz-msgs10 gz-msgs10::core)
      if(TARGET ${t})
        target_link_libraries(FastDroneAero PRIVATE ${t})
      endif()
    endforeach()
    message(STATUS "gz-transport13 + gz-msgs10 발견 -> 힘 화살표를 켭니다")
  else()
    message(STATUS "gz-transport13/gz-msgs10 없음 -> 화살표 없이 빌드합니다")
  endif()
  # gz 는 접두사 없는 파일명(libFastDroneAero.so)을 그대로 찾는다
  set_target_properties(FastDroneAero PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  install(TARGETS FastDroneAero LIBRARY DESTINATION lib)
else()
  message(STATUS "gz-sim8 없음 -> 검증 테스트만 빌드합니다 "
                 "(macOS 개발 환경이면 정상)")
endif()
