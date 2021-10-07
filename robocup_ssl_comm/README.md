# RoboCup SSL Communication package

## Publish topics

`$ ros2 topic pub /commands robocup_ssl_msgs/msg/Commands "{timestamp: 0, isteamyellow: false, robot_commands: [{id: 0, veltangent: 0.0}]}"`

`$ ros2 topic pub /replacement robocup_ssl_msgs/msg/Replacement "{ball: [{x: [0], y: [0], vx: [2], vy: [2]}]}"`

`$ ros2 topic pub /replacement robocup_ssl_msgs/msg/Replacement "{robots:[{id: 1, x: 0.2, y: -0.3, dir: 90, yellowteam: true, turnon: [true]}]}"`