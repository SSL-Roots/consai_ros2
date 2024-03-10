import { Stage, Layer, Circle, Rect, Shape } from "react-konva";
import Field from "./field";
import Robot from "./robot";
import Ball from "./ball";
import DetectionTracked from "./detectionTracked";

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
};

const BirdView = ({ ros }: BirdViewProps) => {
  const canvasSize = {
    width: 13,
    height: 10,
    scale: 50,
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
        <Field ros={ros} />
        <DetectionTracked ros={ros} />
      </Layer>
    </Stage>
  );
};

export default BirdView;
