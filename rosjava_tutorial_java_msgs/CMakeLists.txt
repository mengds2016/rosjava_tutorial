cmake_minimum_required(VERSION 2.8.3)
project(rosjava_tutorial_java_msgs)

find_package(catkin REQUIRED rosjava_build_tools genjava rosjava_tutorial_msgs)

catkin_rosjava_setup()

generate_rosjava_messages(
  PACKAGES
    rosjava_tutorial_msgs
)

catkin_package()

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
