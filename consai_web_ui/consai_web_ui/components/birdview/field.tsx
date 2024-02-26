import { Stage, Layer, Circle, Rect, Line } from "react-konva";

type Vector2D = {
  x: number;
  y: number;
};

type LineSegmentProps = {
  name: string;
  p1: Vector2D;
  p2: Vector2D;
  thickness: number;
};

type CircularArcProps = {
  name: string;
  center: Vector2D;
  radius: number;
  a1: number;
  a2: number;
  thickness: number;
};

type FieldProps = {
  fieldSizeXMm: number;
  fieldSizeYMm: number;
  goalWidthMm: number;
  goalDepthMm: number;
  boundaryWidthMm: number;
  fieldLines: LineSegmentProps[];
  fieldArcs: CircularArcProps[];
};

const Field = ({
  fieldSizeXMm,
  fieldSizeYMm,
  goalWidthMm,
  goalDepthMm,
  boundaryWidthMm,
  fieldLines,
  fieldArcs,
}: FieldProps) => {
  const worldCoordinateToCanvas = (p: Vector2D): Vector2D => {
    const canvasX = p.x + fieldSizeXMm / 2 + boundaryWidthMm;
    const canvasY = p.y + fieldSizeYMm / 2 + boundaryWidthMm;
    return { x: canvasX, y: canvasY };
  };

  const lines = fieldLines.map((line, index) => {
    const p1 = worldCoordinateToCanvas(line.p1);
    const p2 = worldCoordinateToCanvas(line.p2);

    return (
      <Line
        key={index}
        points={[p1.x, p1.y, p2.x, p2.y] as number[]}
        stroke="white"
        strokeWidth={line.thickness}
      />
    );
  });

  const arcs = fieldArcs.map((arc, index) => {
    const center = worldCoordinateToCanvas(arc.center);
    const radius = arc.radius;
    const startAngle = arc.a1;
    const endAngle = arc.a2;
    const clockwise = startAngle < endAngle;

    return (
      <Circle
        key={index}
        x={center.x}
        y={center.y}
        radius={radius}
        stroke="white"
        strokeWidth={arc.thickness}
        startAngle={startAngle}
        endAngle={endAngle}
        clockwise={clockwise}
      />
    );
  });

  return (
    <Layer>
      <Rect
        x={0}
        y={0}
        width={fieldSizeXMm + boundaryWidthMm * 2}
        height={fieldSizeYMm + boundaryWidthMm * 2}
        fill="green"
      />
      {lines}
      {arcs}
    </Layer>
  );
};

export default Field;
