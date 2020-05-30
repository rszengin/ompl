macro(add_ompl_demo demo_name)
  add_executable(${ARGV})
  target_link_libraries(${demo_name}
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY})
endmacro(add_ompl_demo)

add_ompl_demo(demo_RigidBodyPlanning RigidBodyPlanning.cpp)
