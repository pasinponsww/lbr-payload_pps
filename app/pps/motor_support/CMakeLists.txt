add_library(motor_support OBJECT
    dc_motor.cc
)

target_include_directories(motor_support
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}/common/core/periph/drv8245
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/io
    PUBLIC ${CMAKE_SOURCE_DIR}/common/drivers/utils
)


