import { use, useEffect, useState } from "react";
import { Stage, Layer, Circle, Rect, Line, Group } from "react-konva";
import ROSLIB from "roslib";
import Robot from "./robot";
import Ball from "./ball";

type RobotId = {
  id: number;
  team_color: "blue" | "yellow";
};

type TrackedBall = {
  pos: Vector3D;
  vel?: Vector3D;
  visibility?: number;
};

type TrackedRobot = {
  robot_id: RobotId;
  pos: Vector2D;
  orientation: number;
  vel?: Vector2D;
  vel_angular?: number;
  visibility?: number;
};

type TrackedFrame = {
  frame_number: number;
  timestamp: number;
  balls: TrackedBall[];
  robots: TrackedRobot[];
  capabilities?: number[];
};

type DetectionTrackedProps = {
  ros: ROSLIB.Ros | null;
};

const DetectionTracked = ({ ros }: DetectionTrackedProps) => {
  const [trackedFrame, setTrackedFrame] = useState<TrackedFrame>({
    frame_number: 0,
    timestamp: 0,
    balls: [],
    robots: [],
  });

  useEffect(() => {
    if (!ros) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/detection_tracked",
      messageType: "robocup_ssl_msgs/msg/TrackedFrame",
    });

    listener.subscribe((message) => {
      const received: TrackedFrame = {
        frame_number: message.frame_number,
        timestamp: message.timestamp,
        balls: message.balls.map((ball) => {
          return {
            pos: {
              x: ball.pos.x,
              y: ball.pos.y,
              z: ball.pos.z,
            },
            vel: {
              x: ball.vel.x,
              y: ball.vel.y,
              z: ball.vel.z,
            },
            visibility: ball.visibility,
          };
        }),
        robots: message.robots.map((robot) => {
          return {
            robot_id: {
              id: robot.robot_id.id,
              team_color: robot.robot_id.team_color === 1 ? "yellow" : "blue",
            },
            pos: {
              x: robot.pos.x,
              y: robot.pos.y,
            },
            orientation: robot.orientation,
            vel: {
              x: robot.vel.x,
              y: robot.vel.y,
            },
            vel_angular: robot.vel_angular,
            visibility: robot.visibility,
          };
        }),
        capabilities: message.capabilities,
      };

      setTrackedFrame(received);
    });
  }, [ros]);

  const robots = trackedFrame.robots.map((robot, index) => {
    return (
      <Robot
        key={index}
        id={robot.robot_id.id}
        x={robot.pos.x}
        y={robot.pos.y}
        theta={robot.orientation}
        color={robot.robot_id.team_color}
      />
    );
  });

  const balls = trackedFrame.balls.map((ball, index) => {
    return <Ball key={index} x={ball.pos.x} y={ball.pos.y} />;
  });

  console.log(robots);

  return (
    <Group>
      {robots}
      {balls}
    </Group>
  );
};

export default DetectionTracked;
