import { BottomNavigation, BottomNavigationAction, Box, Tab, Tabs } from "@mui/material";
import { TabContext, TabList, TabPanel } from "@mui/lab";
import React, { useState } from "react";

import RestoreIcon from '@mui/icons-material/Restore';
import SimulationControlDrawer from "./SimulationControl";
import SimulationControl from "./SimulationControl";

type SideMenuProps = {
    children?: Array<React.ReactNode>;
};

const SideMenu = ({ children }: SideMenuProps) => {
    const [value, setValue] = useState(0);

    const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
        setValue(newValue);
    };

    return (
        <Box sx={{ overflow: 'auto', height: "100vh" }}>
            <TabContext value={value}>
                <TabList onChange={handleChange}>
                    <Tab label="Simulation Control" value="1" />
                </TabList>

                <TabPanel value="1">
                    {children[0]}
                </TabPanel>
            </TabContext>
        </Box>
    )
};

export default SideMenu; 