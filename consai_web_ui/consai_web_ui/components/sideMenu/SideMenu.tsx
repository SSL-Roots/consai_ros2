import { BottomNavigation, BottomNavigationAction, Box, Tab, Tabs } from "@mui/material";
import { useState } from "react";

import RestoreIcon from '@mui/icons-material/Restore';
import SimulationControlDrawer from "./SimulationControl";

const SideMenu = () => {
    const [value, setValue] = useState(0);

    return (
        <>
            <Box>
                <Tabs
                    value={value}
                    onChange={(_, newValue) => setValue(newValue)}
                    variant="scrollable"
                >
                    <Tab label="Tab 1" />
                    <Tab label="Tab 2" />
                </Tabs>
            </Box>
        </>
    )
};

export default SideMenu; 