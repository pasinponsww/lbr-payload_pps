add_subdirectory(periph)
add_subdirectory(utils)

if (TARGET core)
	target_link_libraries(core INTERFACE utils)
endif()
