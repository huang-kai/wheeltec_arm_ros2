# cmake_minimum_required(VERSION 2.8.3)

# project(mini_mec_six_arm)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

cmake_minimum_required(VERSION 3.10.2)
project(mini_mec_six_arm)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})