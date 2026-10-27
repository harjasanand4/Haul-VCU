add_executable(haul_tests
  test_can.cpp
  test_core.cpp
  test_vcu.cpp
  test_sim.cpp
  test_bus.cpp
  test_rt.cpp)
target_link_libraries(haul_tests PRIVATE haul GTest::gtest_main)
haul_set_warnings(haul_tests)

include(GoogleTest)
# The scenario suite runs ~20 simulated minutes per case; give slow sanitizer builds room.
gtest_discover_tests(haul_tests PROPERTIES TIMEOUT 600 DISCOVERY_TIMEOUT 60)

# Separate binary: it replaces global operator new, which must not leak into the other tests,
# and it only makes sense without sanitizers (they replace the allocator themselves).
if(NOT HAUL_SANITIZE)
  add_executable(haul_no_alloc_test test_no_alloc.cpp)
  target_link_libraries(haul_no_alloc_test PRIVATE haul GTest::gtest_main)
  haul_set_warnings(haul_no_alloc_test)
  # GCC flags malloc/free inside a replacement operator new/delete as "mismatched": that pairing is the point.
  target_compile_options(haul_no_alloc_test PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-mismatched-new-delete>)
  gtest_discover_tests(haul_no_alloc_test)
endif()
