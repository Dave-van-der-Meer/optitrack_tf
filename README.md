# optitrack_tf
TF Broadcaster to be used with vrpn_client_ros for ROS 2

## Usage

Change the name 'tracker_01' to the name of your Optitrack Rigid Body and save it. Then, compile the package with:

```shell-session
$ colcon build --symlink-install --packages-select optitrack_tf
```

Source your workspace with:

```shell-session
$ source install/local_setup.bash
```

Finally, launch the package with:

```shell-session
ros2 launch optitrack_tf optitrack_system_full.launch.py
```

This will start the `vrpn_mocap` launc file with the server IP address pre-set, together with the node that will transform the pose topic from the default coordinate frame from OptiTrack to the default coordinate frame used by ROS, and also the node that will use this transformed pose topic and publish these poses as TF messages. You can change the name of the tracker to be used in the `config/params.yaml` file. Choose the name of the tracker and then add the prefix `vrpn_mocap/`. Inside the node, the nodes will add the `/pose` and `/rectified/pose` suffix automatically.