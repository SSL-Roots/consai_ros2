import { ROBOT_IDS } from "@/utils/constants";
import { PlaceSharp } from "@mui/icons-material";
import { List, ListItem, ListItemIcon, ListItemText, ListSubheader, Switch } from "@mui/material";
import { useState } from "react";
import { BirdViewMouseEvent } from "../birdview/birdview";

type SimulationControlProps = {
    mouseEvent: BirdViewMouseEvent | null;
};

const SimulationControl = ({ mouseEvent }: SimulationControlProps) => {
    return (
        <div>
            <TeamRobotsControl color="blue" mouseEvent={mouseEvent} />
            <TeamRobotsControl color="yellow" mouseEvent={mouseEvent} />
        </div>
    )
};

type TeamRobotsControlProps = {
    color: "blue" | "yellow";
    mouseEvent: BirdViewMouseEvent | null;
};
const TeamRobotsControl = ({ color }: TeamRobotsControlProps) => {
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

    const allControl = () => {
        return (
            <ListItem>
                <ListItemIcon>
                    All
                </ListItemIcon>
                <Switch
                    checked={enableAll}
                    onChange={handleEnableAll}
                />
            </ListItem>
        )
    };

    return (
        <List subheader={<ListSubheader>{color}</ListSubheader>}>
            {allControl()}
            {ROBOT_IDS.map((robotId) => (
                <SingleRobotControl color={color} robotId={robotId} enabled={enabledRobots[robotId]} setEnabled={setEnabled} />
            ))}
        </List>
    )
}


type SingleRobotControlProps = {
    color: "blue" | "yellow";
    robotId: number;
    enabled: boolean;
    setEnabled: (robotId: number, enabled: boolean) => void;
};
const SingleRobotControl = ({ color, robotId, enabled, setEnabled }: SingleRobotControlProps) => {
    const setChecked = (checked: boolean) => {
        setEnabled(robotId, checked);
    }

    return (
        <ListItem>
            <ListItemIcon>
                {robotId}
            </ListItemIcon>
            {/* <ListItemText>
                Robot: {robotId}
            </ListItemText> */}
            <RobotSwitch checked={enabled} setChecked={setChecked} />
            <RobotCursor />
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

const RobotCursor = () => {
    return (
        <PlaceSharp />
    )
}

export default SimulationControl; 