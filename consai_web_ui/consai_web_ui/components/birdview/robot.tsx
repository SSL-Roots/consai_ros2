import { Shape, Text } from "react-konva";

type RobotProps = {
  id: number;
  x: number;
  y: number;
  theta: number; // radian
  color: "blue" | "yellow";
};

const Robot = ({ id, x, y, theta, color }: RobotProps) => {
  const fillColor = color === "blue" ? "#8888ff" : "#ffff33";
  const fontSize = 0.1;

  return (
    <>
      <Shape
        sceneFunc={(context, shape) => {
          context.beginPath();
          context.arc(0, 0, 0.08, Math.PI / 6, -Math.PI / 6);
          context.closePath();
          context.fillStrokeShape(shape);
        }}
        x={x}
        y={y}
        fill={fillColor}
        stroke="black"
        strokeWidth={0.005}
        rotation={theta * (180 / Math.PI)}
      />
      <Text
        x={x - fontSize / 2}
        y={y - fontSize / 2}
        text={id.toString()}
        fontSize={fontSize}
        fill="black"
        align="center"
        verticalAlign="middle"
      />
    </>
  );
};

export default Robot;
