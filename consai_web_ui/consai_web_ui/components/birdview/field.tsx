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
  fieldSizeXMeter: number;
  fieldSizeYMeter: number;
  goalWidthMeter: number;
  goalDepthMeter: number;
  boundaryWidthMeter: number;
  fieldLines: LineSegment[];
  fieldArcs: CircularArc[];
};

type FieldProps = {
  ros: ROSLIB.Ros | null;
};

const Field = ({ ros }: FieldProps) => {
  const [geom, setGeometryFieldSize] = useState<GeometryFieldSize>({
    fieldSizeXMeter: 12,
    fieldSizeYMeter: 9,
    goalWidthMeter: 1,
    goalDepthMeter: 0.5,
    boundaryWidthMeter: 0.3,
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
      const fieldSizeXMeter = message.field.field_length / 1000;
      const fieldSizeYMeter = message.field.field_width / 1000;
      const goalWidthMeter = message.field.goal_width / 1000;
      const goalDepthMeter = message.field.goal_depth / 1000;
      const boundaryWidthMeter = message.field.boundary_width / 1000;
      const fieldLines = message.field.field_lines.map((line) => {
        const p1 = { x: line.p1.x / 1000, y: line.p1.y / 1000 };
        const p2 = { x: line.p2.x / 1000, y: line.p2.y / 1000 };
        return { name: line.name, p1, p2, thickness: line.thickness / 1000 };
      });
      const fieldArcs = message.field.field_arcs.map((arc) => {
        const center = { x: arc.center.x / 1000, y: arc.center.y / 1000 };
        const radius = arc.radius / 1000;
        const a1 = arc.a1;
        const a2 = arc.a2;
        return {
          name: arc.name,
          center,
          radius,
          a1,
          a2,
          thickness: arc.thickness / 1000,
        };
      });
      setGeometryFieldSize({
        fieldSizeXMeter: fieldSizeXMeter,
        fieldSizeYMeter: fieldSizeYMeter,
        goalWidthMeter: goalWidthMeter,
        goalDepthMeter: goalDepthMeter,
        boundaryWidthMeter: boundaryWidthMeter,
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
        x={-geom.fieldSizeXMeter / 2 - geom.boundaryWidthMeter}
        y={-geom.fieldSizeYMeter / 2 - geom.boundaryWidthMeter}
        width={geom.fieldSizeXMeter + geom.boundaryWidthMeter * 2}
        height={geom.fieldSizeYMeter + geom.boundaryWidthMeter * 2}
        fill="green"
      />
      {lines}
      {arcs}
    </Group>
  );
};

export default Field;
