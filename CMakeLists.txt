cmake_minimum_required(VERSION 3.5)
project(b1_navigation)


find_package(ament_cmake REQUIRED)
# find_package(robot_localization REQUIRED)


ament_package()

set(SHARE_DESTINATION share/${PROJECT_NAME})


# Install directories
set(directories launch config)
foreach(dir IN LISTS directories)
    install(
        DIRECTORY ${dir}
        DESTINATION ${SHARE_DESTINATION}
    )
endforeach()

