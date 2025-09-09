# cabot_site_large_room

test world (only gazebo) and maps for cabot testing

## How to build a custom gazebo world and map

0. prepare cabot environment
1. make a world
2. configure your cabot_site pacakge
3. launch cabot with your cabot_site without lidar map
  - launch simulation in cabot-navigation
  - copy your cabot_site pacakge into under `cabot/cabot-navigation/cabot_sites`
  - configure your cabot-navigation .env file
  ```
  CABOT_SITE=cabot_site_large_room
  # do not specify CABOT_INITX, CABOT_INITY, and so on
  # it would be better to start from (0, 0) origin for map alignment
  CABOT_SHOW_GAZEBO_CLIENT=1
  # need to see the robot position in the gazebo world while mapping
  # localization will not work, so you cannot rely on the rviz view
  ```
  - build and launch
  ```
  ./build-docker.sh -w
  ./launch.sh -s
  ```
4. launch mapping system
  - launch mapping in cabot folder
  ```
  ./mapping-launch.sh -s
  ```
5. move your robot in the environment
6. post process the recorded mapping data
  ```
  ./mapping-launch.sh -s -p docker/home/recordings/mapping_<date>
  ```
7. configure your cabot_site with the generated maps
8. configure initial server_data and launch MapService server
9. edit and export topoloty data on the MapService server
10. launch with your cabot_site and go to a destination
