find_package(TBB REQUIRED)

find_package(Eigen3 QUIET)

if(Eigen3_FOUND)
    add_executable(example_eigen example_eigen.cpp)
    target_link_libraries(example_eigen PRIVATE acceleration::acceleration
                                                Eigen3::Eigen TBB::tbb)
    target_compile_definitions(example_eigen PRIVATE ACC_BACKEND_EIGEN)
endif()

find_package(glm QUIET)

if(glm_FOUND)
    add_executable(example_glm example_glm.cpp)
    target_link_libraries(example_glm PRIVATE acceleration::acceleration
                                              glm::glm TBB::tbb)
    target_compile_definitions(example_glm PRIVATE ACC_BACKEND_GLM)
endif()
