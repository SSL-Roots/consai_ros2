import { Stage, Layer, Circle, Rect } from "react-konva";
import Field from "./field";

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
};

const BirdView = ({ ros }: BirdViewProps) => {
  return (
    <Stage
      width={window.innerWidth}
      height={window.innerHeight}
      scale={{ x: 0.05, y: 0.05 }}
    >
      <Field ros={ros} />
    </Stage>
  );
};

export default BirdView;
