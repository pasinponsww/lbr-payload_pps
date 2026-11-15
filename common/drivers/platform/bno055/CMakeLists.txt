add_library(bno055 STATIC
    bno055_imu.cc
)

target_include_directories(bno055 PUBLIC
    .
    ${CMAKE_SOURCE_DIR}/common/core/math
    ${CMAKE_SOURCE_DIR}/common/drivers/utils
)

target_link_libraries(bno055 PUBLIC
    driver
    driver_utils
)
