import { Stage, Layer, Circle, Rect, Shape, Group } from "react-konva";
import Field from "./field";
import Robot from "./robot";
import Ball from "./ball";
import DetectionTracked from "./detectionTracked";
import ROSLIB from "roslib";

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
};

const BirdView = ({ ros }: BirdViewProps) => {
  const publishBallReplacement = (pos: Vector2D) => {
    if (!ros) return;

    const publisher = new ROSLIB.Topic({
      ros: ros,
      name: "/replacement",
      messageType: "robocup_ssl_msgs/msg/Replacement",
    });

    const message = new ROSLIB.Message({
      ball: [{ x: [pos.x], y: [pos.y] }],
      robots: [],
    });

    publisher.publish(message);
  };

  const canvasSize = {
    width: 13,
    height: 10,
    scale: 100,
  };

  return (
    <Stage
      width={window.innerWidth}
      height={window.innerHeight}
      scale={{ x: canvasSize.scale, y: canvasSize.scale }}
    >
      <Layer
        width={canvasSize.width}
        height={canvasSize.height}
        offset={{ x: -canvasSize.width / 2, y: -canvasSize.height / 2 }}
      >
        <Group
          rotation={0}
          onDblClick={(e) => {
            console.log(e.currentTarget.getRelativePointerPosition());
            const pos: Vector2D = {
              x: e.currentTarget.getRelativePointerPosition().x,
              y: e.currentTarget.getRelativePointerPosition().y,
            };
            publishBallReplacement(pos);
          }}
        >
          <Field ros={ros} />
          <DetectionTracked ros={ros} />
        </Group>
      </Layer>
    </Stage>
  );
};

export default BirdView;
