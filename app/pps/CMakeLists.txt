add_library(pps
    pps.cc
    $<TARGET_OBJECTS:motor_support>
)

target_include_directories(pps PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_support
    ${CMAKE_SOURCE_DIR}/common/core/periph/drv8245
)
target_link_libraries(pps PUBLIC bno055 helpers drv8245)



add_subdirectory(motor_support)
add_subdirectory(helpers)
add_subdirectory(pps_bsp)

add_executable(pps_app main.cc ../../syscalls.c $<TARGET_OBJECTS:helpers> $<TARGET_OBJECTS:motor_support>)
target_link_libraries(pps_app PRIVATE pps pps_bsp drv8245)
set_target_properties(pps_app PROPERTIES LINKER_LANGUAGE CXX)
