import { ROBOT_IDS } from "@/utils/constants";
import { PlaceSharp } from "@mui/icons-material";
import { List, ListItem, ListItemIcon, ListItemText, ListSubheader, Switch } from "@mui/material";
import { useState } from "react";

const SimulationControl = () => {
    return (
        <div>
            <List subheader={<ListSubheader>Blue</ListSubheader>}>
                {ROBOT_IDS.map((robotId) => (
                    <RobotControl color="blue" robotId={robotId} />
                ))}
            </List>
            <List subheader={<ListSubheader>Yellow</ListSubheader>}>
                {ROBOT_IDS.map((robotId) => (
                    <RobotControl color="yellow" robotId={robotId} />
                ))}
            </List>
        </div>
    )
};

type RobotControlProps = {
    color: "blue" | "yellow";
    robotId: number;
};
const RobotControl = ({ color, robotId }: RobotControlProps) => {
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