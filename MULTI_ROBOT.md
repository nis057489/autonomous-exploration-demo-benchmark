# Turtlebot3 Multi-Robot SLAM

## Launch from your laptop

`./launch_multi_robot.sh` SSHes into all 3 robots below and runs each of
their commands inside a detached tmux session named `exploration`.

`./stop_multi_robot.sh` sends Ctrl-C into each robot's session for a clean
shutdown (`--force` to kill the sessions outright instead).

## Robot1

```sh
 git pull && ./launch_real_hardware.sh   --robot-id robot1   --robot-offset-x 0.0   --robot-offset-y 0.0   --robot-offset-yaw 0.0 --local-bringup
```

## Robot 2

```sh
 git pull && ./launch_real_hardware.sh   --robot-id robot2   --robot-offset-x 5.7  --robot-offset-y 1.5   --robot-offset-yaw 0.0 --local-bringup
```

## Robot 3

```sh
git pull && ./launch_real_hardware.sh   --robot-id robot3   --robot-offset-x 0.0   --robot-offset-y -3.6   --robot-offset-yaw 0.0 --local-bringup
```
