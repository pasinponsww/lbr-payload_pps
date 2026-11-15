add_library(pps_bsp STATIC
    l476.cc
)

target_link_libraries(pps_bsp PUBLIC pps hal bno055 core)
