import { Circle, Shape, Text } from "react-konva";

type BallProps = {
  x: number;
  y: number;
};

const Ball = ({ x, y }: BallProps) => {
  return (
    <Circle
      x={x}
      y={y}
      radius={21}
      fill="orange"
      stroke="black"
      strokeWidth={5}
    />
  );
};

export default Ball;
