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
    const allControl = () => {
        return (
            <ListItem>
                <ListItemIcon>
                    All
                </ListItemIcon>
                <RobotSwitch />
            </ListItem>
        )
    };

    return (
        <List subheader={<ListSubheader>{color}</ListSubheader>}>
            {allControl()}
            {ROBOT_IDS.map((robotId) => (
                <SingleRobotControl color={color} robotId={robotId} />
            ))}
        </List>
    )
}


type SingleRobotControlProps = {
    color: "blue" | "yellow";
    robotId: number;
};
const SingleRobotControl = ({ color, robotId }: SingleRobotControlProps) => {
    return (
        <ListItem>
            <ListItemIcon>
                {robotId}
            </ListItemIcon>
            {/* <ListItemText>
                Robot: {robotId}
            </ListItemText> */}
            <RobotSwitch />
            <RobotCursor />
        </ListItem>
    )
}

const RobotSwitch = () => {
    const [checked, setChecked] = useState(false);

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