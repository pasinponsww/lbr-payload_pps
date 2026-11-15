add_executable(pps_sim
    main.cc
    plant.cc
    simulator.cc
)

target_link_libraries(pps_sim PRIVATE pps)
