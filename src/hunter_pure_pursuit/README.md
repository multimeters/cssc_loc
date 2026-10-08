# Hunter Pure Pursuit

This ROS 2 Humble package follows four fixed map-frame waypoints in config/route.yaml using Hunter /cmd_vel. After reaching waypoint 4, it continues from waypoint 1 and repeats the route.

It subscribes to /localization/kinematic_state and transient-local /localization/fusion_status. Nonzero commands are sent only while localization is fresh and fusion mode is FUSED; otherwise the node continually publishes zero velocity. At startup it resumes after a waypoint when the current pose is already near that waypoint.

Hunter SE Ackermann constraints limit curvature using wheelbase, steering angle, and minimum turning radius. The config also caps forward and reverse speed, yaw rate, lateral acceleration, and acceleration ramps.

One-command start on the remote robot:
cd /home/user/cssc_loc
./scripts/start_hunter_pure_pursuit.sh

The script starts the Hunter chassis driver from config/hunter_base.yaml when one is not already running, waits for /hunter_odom, then starts tracking. Localization must already be running on ROS domain 42 and report FUSED before the controller sends motion commands. If the driver was already running, the script reuses it and leaves it running when the script exits. Press Ctrl+C to stop tracking; if this script started the driver, it also stops that driver.
