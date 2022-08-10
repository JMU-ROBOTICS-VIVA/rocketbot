# Rocket Bot

A simulated Rocket with a ROS 2 interface.  This is intended to serve
as an alternative to turtlesim for exploring topics and messages.

## Non-ROS Dependencies

* pygame

## Usage

```
ros2 run rocketbot rocketbot_node
```

## ROS 2 API

Publishes to:
* `/location`      (geometry_msgs/Point)
* `/target `       (geometry_msgs/Point)  Latest mouse click position
                                       (published continously)
* `/target_event`  (geometry_msgs/Point)  Latest mouse click position
                                       (only published when click occurs)


Subscribes to:
* `/thrust`    (geometry_msgs/Vector3)

(0, 0, 0) is at the lower left edge of the screen.  The x axis points
to the right and the y axis points up.  Thrust along the z axis will
be ignored.  Negative thrusts along the y axis will be ignored.

