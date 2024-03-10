import { Stage, Layer, Circle, Rect, Shape } from "react-konva";
import Field from "./field";

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
};

const BirdView = ({ ros }: BirdViewProps) => {
  const canvasSize = {
    width: 13000,
    height: 10000,
    scale: 0.1,
  };
  return (
    <Stage
      width={canvasSize.width}
      height={canvasSize.height}
      offset={{ x: -canvasSize.width / 2, y: -canvasSize.height / 2 }}
      scale={{ x: canvasSize.scale, y: canvasSize.scale }}
    >
      <Layer rotation={0}>
        <Field ros={ros} />
        <Shape
          sceneFunc={(context, shape) => {
            context.beginPath();
            context.arc(0, 0, 90, Math.PI / 4, -Math.PI / 4);
            context.closePath();
            context.fillStrokeShape(shape);
          }}
          fill="yellow"
          stroke="black"
          strokeWidth={10}
        />
      </Layer>
    </Stage>
  );
};

export default BirdView;
