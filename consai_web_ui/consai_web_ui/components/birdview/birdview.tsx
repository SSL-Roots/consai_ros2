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

type CursorMode = {
  type: "ball" | "robot" | "none";
  team: "blue" | "yellow" | null;
  id: number | null;
};

const BirdView = ({ ros, setMouseEvent }: BirdViewProps) => {
  const [cursorMode, setCursorMode] = React.useState<CursorMode>({
    type: "none",
    team: null,
    id: null,
  });
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

  const publishBallReplacement = (pos: Vector2D) => {
    if (!ros) return;
    if (cursorMode.type === "none") return;

    const publisher = new ROSLIB.Topic({
      ros: ros,
      name: "/replacement",
      messageType: "robocup_ssl_msgs/msg/Replacement",
    });

    const generateMessage = (): ROSLIB.Message | null => {
      if (cursorMode.type === "ball") {
        return new ROSLIB.Message({
          ball: [{ x: [pos.x], y: [pos.y] }],
          robots: [],
        });
      }

      if (cursorMode.team === null || cursorMode.id === null) return null;

      if (cursorMode.type === "robot") {
        return new ROSLIB.Message({
          ball: [],
          robots: [
            {
              x: pos.x,
              y: pos.y,
              id: cursorMode.id,
              yellowteam: cursorMode.team === "yellow",
            },
          ],
        });
      }

      return null;
    };

    const message = generateMessage();
    if (message) {
      publisher.publish(message);
    }
  };

  const canvasSize = {
    width: 13,
    height: 10,
  };

  const scale = windowWidth / canvasSize.width;

  return (
    <div id="birdViewContainer">
      <CursorSelector setCursorMode={setCursorMode} />
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
              publishBallReplacement(pos);
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

type CursorSelectorProps = {
  setCursorMode: React.Dispatch<React.SetStateAction<CursorMode>>;
};
const CursorSelector = ({ setCursorMode }: CursorSelectorProps) => {
  const [selection, setSelection] = React.useState<string>("none");

  const handleClick = (
    event: React.MouseEvent<HTMLElement>,
    value: string | null
  ) => {
    if (!value) return;

    const cursorType = () => {
      if (value === "none") return "none";
      if (value === "ball") return "ball";
      return "robot";
    };

    const cursorTeam = () => {
      if (value === "none" || value === "ball") return null;
      return value.includes("blue") ? "blue" : "yellow";
    };

    const cursorId = () => {
      if (value === "none" || value === "ball") return null;
      return parseInt(value.split("-")[1]);
    };

    const cursorMode: CursorMode = {
      type: cursorType(),
      team: cursorTeam(),
      id: cursorId(),
    };

    setCursorMode(cursorMode);
    setSelection(value);
  };

  const buttonBaseStyle = css({
    width: "2em",
    height: "2em",
    textAlign: "center",
    lineHeight: "2em",
    borderRadius: "50%",
  });
  const blueStyle = css(buttonBaseStyle, {
    backgroundColor: "#5FB2DB",
  });
  const yellowStyle = css(buttonBaseStyle, {
    backgroundColor: "#DED943",
  });
  const ballStyle = css(buttonBaseStyle, {
    backgroundColor: "#DB7A44",
  });

  const ids = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"];
  const blueButtons = ids.map((id, index) => (
    <ToggleButton key={index} value={`blue-${id}`} aria-label={`blue-${id}`}>
      <div css={blueStyle}>{id}</div>
    </ToggleButton>
  ));
  const yellowButtons = ids.map((id, index) => (
    <ToggleButton
      key={index}
      value={`yellow-${id}`}
      aria-label={`yellow-${id}`}
    >
      <div css={yellowStyle}>{id}</div>
    </ToggleButton>
  ));

  return (
    <>
      <Box component="div" sx={{ overflow: "auto" }}>
        <ToggleButtonGroup
          value={selection}
          exclusive
          onChange={handleClick}
          aria-label="cursor selector"
        >
          <ToggleButton value={"ball"} aria-label={"ball"}>
            <div css={ballStyle}>B</div>
          </ToggleButton>

          {blueButtons}
          {yellowButtons}
        </ToggleButtonGroup>
      </Box>
    </>
  );
};
export default BirdView;
