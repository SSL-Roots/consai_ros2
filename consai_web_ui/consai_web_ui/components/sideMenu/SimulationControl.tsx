import { ROBOT_IDS } from "@/utils/constants";
import { PlaceSharp } from "@mui/icons-material";
import { Box, Button, List, ListItem, ListItemIcon, ListItemText, ListSubheader, Switch, ToggleButton } from "@mui/material";
import { useEffect, useState } from "react";
import { BirdViewMouseEvent } from "../birdview/birdview";
import ROSLIB from "roslib";
import { publishReplacement } from "@/utils/consaiInterfaces";

type SimulationControlProps = {
    ros: ROSLIB.Ros;
    mouseEvent: BirdViewMouseEvent | null;
};

const SimulationControl = ({ ros, mouseEvent }: SimulationControlProps) => {
    type SelectedCursor = {
        type: "robot" | "ball" | null;
        color: "blue" | "yellow" | null;
        robotId: number | null;
    };
    const [selectedCursor, setSelectedCursor] = useState<SelectedCursor>({
        type: null,
        color: null,
        robotId: null
    });

    useEffect(() => {
        if (mouseEvent?.type === "dblclick") {
            if (selectedCursor.type === null) return;
            // 再配置
            publishReplacement(ros, selectedCursor.type, selectedCursor.color, selectedCursor.robotId, { x: mouseEvent.x0, y: mouseEvent.y0 });
        }
    }, [mouseEvent]);

    const selectedBall = (): boolean => {
        if (selectedCursor.type !== "ball") return false;
        return true;
    }
    const selectedBlueRobot = (): number | null => {
        if (selectedCursor.type !== "robot") return null;
        if (selectedCursor.color !== "blue") return null;
        if (selectedCursor.robotId === null) return null;
        return selectedCursor.robotId;
    }
    const selectedYellowRobot = (): number | null => {
        if (selectedCursor.type !== "robot") return null;
        if (selectedCursor.color !== "yellow") return null;
        if (selectedCursor.robotId === null) return null;
        return selectedCursor.robotId;
    }
    const selectBall = () => {
        setSelectedCursor({
            type: "ball",
            color: null,
            robotId: null
        });
    }
    const selectRobot = (color: "blue" | "yellow", robotId: number) => {
        setSelectedCursor({
            type: "robot",
            color: color,
            robotId: robotId
        });
    }

    return (
        <div>
            <BallControl selected={selectedBall()} selectBall={selectBall} />
            <TeamRobotsControl color="blue" mouseEvent={mouseEvent} selectedRobotId={selectedBlueRobot()} selectRobot={selectRobot} />
            <TeamRobotsControl color="yellow" mouseEvent={mouseEvent} selectedRobotId={selectedYellowRobot()} selectRobot={selectRobot} />
        </div>
    )
};


type BallControlProps = {
    selected: boolean;
    selectBall: () => void;
};
const BallControl = ({ selected, selectBall }: BallControlProps) => {
    return (
        <List>
            <ListItem>
                <ListItemText>
                    Ball
                </ListItemText>
                <RobotCursorButton selected={selected} onChangeFunc={selectBall} />
            </ListItem>
        </List>
    )
}

type TeamRobotsControlProps = {
    color: "blue" | "yellow";
    mouseEvent: BirdViewMouseEvent | null;
    selectedRobotId: number | null;
    selectRobot: (color: "blue" | "yellow", robotId: number) => void;
};
const TeamRobotsControl = ({ color, mouseEvent, selectedRobotId, selectRobot }: TeamRobotsControlProps) => {
    const [enabledRobots, setEnabledRobots] = useState<{ [key: number]: boolean }>(
        // ROBOT_IDS の各要素をキーとし、true を値とするオブジェクトを生成
        Object.fromEntries(ROBOT_IDS.map((robotId) => [robotId, true]))
    );
    const [enableAll, setEnableAll] = useState(true);

    const handleEnableAll = (event: React.ChangeEvent<HTMLInputElement>) => {
        setEnableAll(event.target.checked);
        setEnabledRobots(
            Object.fromEntries(ROBOT_IDS.map((robotId) => [robotId, event.target.checked]))
        );
    }

    const setEnabled = (robotId: number, enabled: boolean) => {
        setEnabledRobots({
            ...enabledRobots,
            [robotId]: enabled
        });
    }
    const selectColoredRobot = (robotId: number) => {
        selectRobot(color, robotId);
    }

    const allControl = () => {
        return (
            <ListItem>
                <ListItemText>
                    All
                </ListItemText>
                <Switch
                    checked={enableAll}
                    onChange={handleEnableAll}
                />
                <Box sx={{ visibility: "hidden" }}> {/* 列を揃えるためのダミー */}
                    <RobotCursorButton selected={false} onChangeFunc={() => { }} />
                </Box>
            </ListItem>
        )
    };

    return (
        <List subheader={<ListSubheader>{color}</ListSubheader>}>
            {allControl()}
            {ROBOT_IDS.map((robotId) => (
                <SingleRobotControl color={color} robotId={robotId} enabled={enabledRobots[robotId]} setEnabled={setEnabled} selected={selectedRobotId === robotId ? true : false} selectRobot={selectColoredRobot} />
            ))}
        </List>
    )
}


type SingleRobotControlProps = {
    color: "blue" | "yellow";
    robotId: number;
    enabled: boolean;
    setEnabled: (robotId: number, enabled: boolean) => void;
    selected: boolean;
    selectRobot: (robotId: number) => void;
};
const SingleRobotControl = ({ color, robotId, enabled, setEnabled, selected, selectRobot }: SingleRobotControlProps) => {
    const setChecked = (checked: boolean) => {
        setEnabled(robotId, checked);
    }
    const selectFunc = () => {
        selectRobot(robotId);
    }

    return (
        <ListItem>
            <ListItemText>
                {robotId}
            </ListItemText>
            <RobotSwitch checked={enabled} setChecked={setChecked} />
            <RobotCursorButton selected={selected} onChangeFunc={selectFunc} />
        </ListItem>
    )
}

type RobotSwitchProps = {
    checked: boolean;
    setChecked: (checked: boolean) => void;
};
const RobotSwitch = ({ checked, setChecked }: RobotSwitchProps) => {
    const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        setChecked(event.target.checked);
    };

    return (
        <Switch
            checked={checked}
            onChange={handleChange}
        />
    )
}

type RobotCursorButtonProps = {
    selected: boolean;
    onChangeFunc: () => void;
};

const RobotCursorButton = ({ selected, onChangeFunc }: RobotCursorButtonProps) => {
    return (
        <ToggleButton
            size="small"
            value="check"
            selected={selected}
            onChange={onChangeFunc}
        >
            <PlaceSharp />
        </ToggleButton>
    )
}

export default SimulationControl; 