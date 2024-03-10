import { Stage, Layer, Circle, Rect } from "react-konva";
import Field from "./field";

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
};

const BirdView = ({ ros }: BirdViewProps) => {
  const canvasSize = {
    width: 13000,
    height: 10000,
    scale: 0.05,
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
      </Layer>
    </Stage>
  );
};

export default BirdView;
