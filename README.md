# openvslam_ros

[OpenVSLAM](https://github.com/OpenVSLAM-Community/openvslam)'s ROS package.

# Install and start

```docker build -t openvslam-ros-desktop -f Dockerfile .```

```xhost +local:```

```docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro --ipc=host --network=host --runtime=nvidia openvslam-ros-desktop```

```curl -sL "https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow```

```source devel/setup.bash``` 

```export ROS_MASTER_URI=http://10.0.1.11:11311/```

```export ROS_IP=10.0.1.11```

```export ROS_HOSTNAME=10.0.1.11```

```docker cp equirectangular.yaml 21412:/openvslam/ros/1/```

```rosrun openvslam_ros run_slam -v orb_vocab.fbow -c equirectangular.yaml```
