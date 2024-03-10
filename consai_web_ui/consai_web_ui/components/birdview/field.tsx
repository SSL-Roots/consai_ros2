import { use, useEffect, useState } from "react";
import { Stage, Layer, Circle, Rect, Line, Group } from "react-konva";
import ROSLIB from "roslib";

type Vector2D = {
  x: number;
  y: number;
};

type LineSegment = {
  name: string;
  p1: Vector2D;
  p2: Vector2D;
  thickness: number;
};

type CircularArc = {
  name: string;
  center: Vector2D;
  radius: number;
  a1: number;
  a2: number;
  thickness: number;
};

type GeometryFieldSize = {
  fieldSizeXMm: number;
  fieldSizeYMm: number;
  goalWidthMm: number;
  goalDepthMm: number;
  boundaryWidthMm: number;
  fieldLines: LineSegment[];
  fieldArcs: CircularArc[];
};

type FieldProps = {
  ros: ROSLIB.Ros | null;
};

const Field = ({ ros }: FieldProps) => {
  const [geom, setGeometryFieldSize] = useState<GeometryFieldSize>({
    fieldSizeXMm: 12000,
    fieldSizeYMm: 9000,
    goalWidthMm: 1000,
    goalDepthMm: 500,
    boundaryWidthMm: 300,
    fieldLines: [],
    fieldArcs: [],
  });

  useEffect(() => {
    if (!ros) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/geometry",
      messageType: "robocup_ssl_msgs/msg/GeometryData",
    });

    listener.subscribe((message) => {
      const fieldSizeXMm = message.field.field_length;
      const fieldSizeYMm = message.field.field_width;
      const goalWidthMm = message.field.goal_width;
      const goalDepthMm = message.field.goal_depth;
      const boundaryWidthMm = message.field.boundary_width;
      const fieldLines = message.field.field_lines.map((line) => {
        const p1 = { x: line.p1.x, y: line.p1.y };
        const p2 = { x: line.p2.x, y: line.p2.y };
        return { name: line.name, p1, p2, thickness: line.thickness };
      });
      const fieldArcs = message.field.field_arcs.map((arc) => {
        const center = { x: arc.center.x, y: arc.center.y };
        const radius = arc.radius;
        const a1 = arc.a1;
        const a2 = arc.a2;
        return {
          name: arc.name,
          center,
          radius,
          a1,
          a2,
          thickness: arc.thickness,
        };
      });
      setGeometryFieldSize({
        fieldSizeXMm,
        fieldSizeYMm,
        goalWidthMm,
        goalDepthMm,
        boundaryWidthMm,
        fieldLines,
        fieldArcs,
      });
    });
  }, [ros]);

  const lines = geom.fieldLines.map((line, index) => {
    return (
      <Line
        key={index}
        points={[line.p1.x, line.p1.y, line.p2.x, line.p2.y] as number[]}
        stroke="white"
        strokeWidth={line.thickness}
      />
    );
  });

  const arcs = geom.fieldArcs.map((arc, index) => {
    const radius = arc.radius;
    const startAngle = arc.a1;
    const endAngle = arc.a2;
    const clockwise = startAngle < endAngle;

    return (
      <Circle
        key={index}
        x={arc.center.x}
        y={arc.center.y}
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
    <Group>
      <Rect
        x={-geom.fieldSizeXMm / 2 - geom.boundaryWidthMm}
        y={-geom.fieldSizeYMm / 2 - geom.boundaryWidthMm}
        width={geom.fieldSizeXMm + geom.boundaryWidthMm * 2}
        height={geom.fieldSizeYMm + geom.boundaryWidthMm * 2}
        fill="green"
      />
      {lines}
      {arcs}
    </Group>
  );
};

export default Field;
