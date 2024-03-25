import { Stage, Layer, Circle, Rect, Shape, Group } from "react-konva";
import Field from "./field";
import Robot from "./robot";
import Ball from "./ball";
import DetectionTracked from "./detectionTracked";
import ROSLIB from "roslib";

import * as React from "react";
import FormatAlignLeftIcon from "@mui/icons-material/FormatAlignLeft";
import FormatAlignCenterIcon from "@mui/icons-material/FormatAlignCenter";
import FormatAlignRightIcon from "@mui/icons-material/FormatAlignRight";
import FormatAlignJustifyIcon from "@mui/icons-material/FormatAlignJustify";
import ToggleButton from "@mui/material/ToggleButton";
import ToggleButtonGroup from "@mui/material/ToggleButtonGroup";

import { css } from "@emotion/react";
import { Box } from "@mui/material";

export type BirdViewMouseEvent = {
  type: "click" | "dblclick" | "drag";
  x0: number;
  y0: number;
  x1: number | null;
  y1: number | null;
};

type BirdViewProps = {
  ros: ROSLIB.Ros | null;
  setMouseEvent: React.Dispatch<React.SetStateAction<BirdViewMouseEvent | null>>;
};


const BirdView = ({ ros, setMouseEvent }: BirdViewProps) => {
  const [windowWidth, setWindowWidth] = React.useState<number>(
    window.innerWidth
  );

  React.useEffect(() => {
    const getContainerSize = () => {
      const container = document.querySelector('#birdViewContainer');
      const containerWidth = container?.offsetWidth;
      const containerHeight = container?.offsetHeight;
      return { containerWidth, containerHeight };
    }

    window.addEventListener("resize", () => {
      const containerSize = getContainerSize();
      setWindowWidth(containerSize.containerWidth || 0);
    });

    // 初回レンダリング時にも実行
    const containerSize = getContainerSize();
    setWindowWidth(containerSize.containerWidth || 0);
  }, []);

  const canvasSize = {
    width: 13,
    height: 10,
  };

  const scale = windowWidth / canvasSize.width;

  return (
    <div id="birdViewContainer">
      <Stage
        width={windowWidth}
        height={window.innerHeight}
        scale={{ x: scale, y: scale }}
      >
        <Layer offset={{ x: -canvasSize.width / 2, y: -canvasSize.height / 2 }}>
          <Group
            rotation={0}
            onDblClick={(e) => {
              const pos: Vector2D = {
                x: e.currentTarget.getRelativePointerPosition().x,
                y: e.currentTarget.getRelativePointerPosition().y,
              };
              setMouseEvent({
                type: "dblclick",
                x0: pos.x,
                y0: pos.y,
                x1: null,
                y1: null,
              });
            }}
          >
            <Field ros={ros} />
            <DetectionTracked ros={ros} />
          </Group>
        </Layer>
      </Stage>
    </div>
  );
};

export default BirdView;
