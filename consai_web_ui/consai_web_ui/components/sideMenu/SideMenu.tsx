import { BottomNavigation, BottomNavigationAction, Box, Tab, Tabs } from "@mui/material";
import { TabContext, TabList, TabPanel } from "@mui/lab";
import React, { useState } from "react";

import RestoreIcon from '@mui/icons-material/Restore';
import SimulationControlDrawer from "./SimulationControl";
import SimulationControl from "./SimulationControl";

export type Child = {
    label: string
    value: string
    component: React.ReactNode
};

type SideMenuProps = {
    children?: Array<Child>
};

const SideMenu = ({ children }: SideMenuProps) => {
    const [value, setValue] = useState(children ? children[0].value : "");

    const handleChange = (event: React.ChangeEvent<{}>, newValue: string) => {
        setValue(newValue);
    };

    const tabs = children?.map((child) => {
        return (
            <Tab label={child.label} value={child.value} />
        )
    });
    const tabPanels = children?.map((child) => {
        return (
            <TabPanel value={child.value}>
                {child.component}
            </TabPanel>
        )
    });

    return (
        <Box sx={{ overflow: 'auto', height: "100vh" }}>
            <TabContext value={value}>
                <TabList onChange={handleChange}>
                    {tabs}
                </TabList>
                {tabPanels}
            </TabContext>
        </Box>
    )
};

export default SideMenu; 