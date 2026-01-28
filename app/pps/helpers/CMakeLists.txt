
add_library(helpers OBJECT
    pps_helpers.cc
)

target_include_directories(helpers
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
    PUBLIC ${CMAKE_SOURCE_DIR}/common/core/periph/drv8245
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/io
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/utils
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/bus
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/platform/bno055
    PUBLIC ${CMAKE_SOURCE_DIR}/common/core/math
    PUBLIC ${CMAKE_SOURCE_DIR}/common/core
)


