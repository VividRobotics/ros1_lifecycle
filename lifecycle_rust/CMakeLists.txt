cmake_minimum_required(VERSION 3.0.2)
project(lifecycle_rust)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  lifecycle_msgs
  # roslint
)

# TODO(lucasw) needed for kinetic
set(CMAKE_CXX_STANDARD 17)

find_package(Corrosion REQUIRED)
corrosion_import_crate(MANIFEST_PATH Cargo.toml)

# Not ready for this
# roslint_rust()

catkin_package(
  # CATKIN_DEPENDS roslint
)

function(rust_target target_name)
  add_custom_target(${target_name}_copy_to_devel ALL
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target_name}> ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/${target_name}
  )
  add_dependencies(${target_name}_copy_to_devel ${target_name})
  corrosion_install(
    TARGETS ${target_name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction(rust_target)

# TODO(lucasw) will there be any non-test targets here?
# rust_target(foo)
