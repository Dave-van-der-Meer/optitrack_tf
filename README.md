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
ros2 launch optitrack_tf optitrack_tf.launch.py
```
