import { Stage, Layer, Circle, Rect } from "react-konva";
import Field from "./field";

const BirdView = () => {
  return (
    <Stage
      width={window.innerWidth}
      height={window.innerHeight}
      scale={{ x: 0.05, y: 0.05 }}
    >
      <Field
        fieldSizeXMm={12000}
        fieldSizeYMm={9000}
        goalWidthMm={1000}
        goalDepthMm={500}
        boundaryWidthMm={300}
        fieldLines={[
          //   {
          //     name: "boundary",
          //     p1: { x: 0, y: 0 },
          //     p2: { x: 12000, y: 0 },
          //     thickness: 10,
          //   },
          //   {
          //     name: "boundary",
          //     p1: { x: 12000, y: 0 },
          //     p2: { x: 12000, y: 9000 },
          //     thickness: 10,
          //   },
          //   {
          //     name: "boundary",
          //     p1: { x: 12000, y: 9000 },
          //     p2: { x: 0, y: 9000 },
          //     thickness: 10,
          //   },
          //   {
          //     name: "boundary",
          //     p1: { x: 0, y: 9000 },
          //     p2: { x: 0, y: 0 },
          //     thickness: 10,
          //   },
          {
            name: "center",
            p1: { x: -6000, y: 0 },
            p2: { x: 6000, y: 0 },
            thickness: 10,
          },
          {
            name: "halfwayline",
            p1: { x: 0, y: -4500 },
            p2: { x: 0, y: 4500 },
            thickness: 10,
          },
        ]}
        fieldArcs={[
          {
            name: "centercircle",
            center: { x: 0, y: 0 },
            radius: 1000,
            a1: 0,
            a2: 360,
            thickness: 10,
          },
        ]}
      />
    </Stage>
  );
};

export default BirdView;
