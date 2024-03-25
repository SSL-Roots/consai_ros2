import { PlaceSharp } from "@mui/icons-material";
import { List, ListItem, ListItemIcon, ListItemText, ListSubheader, Switch } from "@mui/material";
import { useState } from "react";

const SimulationControl = () => {
    const robotIds = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    return (
        <div>
            <List subheader={<ListSubheader>Blue</ListSubheader>}>
                {robotIds.map((robotId) => (
                    <RobotControl color="blue" robotId={robotId} />
                ))}
            </List>
            <List subheader={<ListSubheader>Yellow</ListSubheader>}>
                {robotIds.map((robotId) => (
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