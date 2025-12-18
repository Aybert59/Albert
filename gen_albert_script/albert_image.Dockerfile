FROM yahboomtechnology/ros-noetic:3.0.4
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
	&& apt-get update && apt-get upgrade -y \
	&& apt install ros-noetic-slam-toolbox -y \
	&& apt install ros-noetic-tf2-web-republisher -y
