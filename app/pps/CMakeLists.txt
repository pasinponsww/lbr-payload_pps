add_library(pps STATIC
    core/console.cc
    core/pps.cc
    core/rtask.cc
    fdir/fdir.cc
    fdir/recovery.cc
    motion/motor.cc
    motion/pid.cc
    motion/travel.cc
    sensor/debounce.cc
    sensor/flight.cc
    sensor/sensors.cc
)

target_include_directories(pps PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common/core/math
)

target_link_libraries(pps PUBLIC drv8245 bno055 driver)

if (TARGET_DEVICE STREQUAL "NATIVE")
    add_subdirectory(sim)
    return()
endif()

add_subdirectory(bsp)

add_executable(pps_app main.cc ${CMAKE_SOURCE_DIR}/syscalls.c ${STARTUP_FILE})
target_link_libraries(pps_app PRIVATE pps pps_bsp)
set_target_properties(pps_app PROPERTIES LINKER_LANGUAGE CXX)

target_link_options(pps_app PRIVATE
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segments
    -Wl,-Map=pps_app.map,--cref
)

add_custom_command(TARGET pps_app POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:pps_app> pps_app.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:pps_app> pps_app.bin
    COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley $<TARGET_FILE:pps_app>
)
