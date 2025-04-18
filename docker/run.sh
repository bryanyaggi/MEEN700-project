#export ROS_MASTER_URI=http://192.168.132.31:11311
#export ROS_IP=192.168.132.15
docker compose -f local.yml run --rm --name ros-radio-localization ros-noetic
