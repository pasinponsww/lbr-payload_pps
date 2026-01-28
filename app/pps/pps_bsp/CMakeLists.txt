add_library(pps_bsp STATIC
    l476_board.cc)

target_include_directories(pps_bsp PUBLIC 
  .
  ../
  ${CMAKE_SOURCE_DIR}/common/drivers/platform/bno055
  ${CMAKE_SOURCE_DIR}/common/drivers/utils
  ${CMAKE_SOURCE_DIR}/common/core/math
  ${CMAKE_SOURCE_DIR}/common/core/periph/drv8245
)

target_link_libraries(pps_bsp PUBLIC core helpers)