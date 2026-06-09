enable_testing()

include(GoogleTest)

add_subdirectory(algorithms)
if(PACKINGSOLVER_BUILD_BOX_SOLVERS)
    add_subdirectory(box)
    add_subdirectory(boxstacks)
endif()
add_subdirectory(irregular)
add_subdirectory(onedimensional)
add_subdirectory(rectangle)
add_subdirectory(rectangleguillotine)
