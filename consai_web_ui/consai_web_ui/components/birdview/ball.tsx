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
      radius={0.021}
      fill="orange"
      stroke="black"
      strokeWidth={0.001}
    />
  );
};

export default Ball;
