macro(add_to_compilation demo_name)
  add_executable(${ARGV})
  target_link_libraries(${demo_name}
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY})
endmacro(add_to_compilation)

add_to_compilation(RigidBodyPlanning RigidBodyPlanning.cpp)
add_to_compilation(GeometricCarPlanning GeometricCarPlanning.cpp)