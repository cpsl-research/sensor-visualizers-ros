# Sensor Visualizers

## QoS Settings
Best practices are:
- All publishers should be reliable with allowable override
- All subscribers should be best effort with allowable override

To see general QoS demos, [see this page](https://github.com/ros2/demos/tree/a66f0e894841a5d751bce6ded4983acb780448cf/quality_of_service_demo#qos-overrides).

For an example of overriding QoS settings for a rosbag replay, [see this guide](https://docs.ros.org/en/iron/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html).