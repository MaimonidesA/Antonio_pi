


docker run --privileged -it \
 --name ros2_humble -h ros2dk \
 -v $PWD/antonio_ws:/home/ros/antonio_ws \
 --network host \
 --ipc=host \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -e DISPLAY=$DISPLAY \
 --rm antonio_image
 

#--user ros \

#'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc
#'echo 'export CYCLONEDDS_URI=$(pwd)/cyclonedds-profile.xml' >> /root/.bashrc
