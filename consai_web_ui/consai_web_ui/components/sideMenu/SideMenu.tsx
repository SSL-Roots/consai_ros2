import { BottomNavigation, BottomNavigationAction, Box, Tab, Tabs } from "@mui/material";
import { TabContext, TabList, TabPanel } from "@mui/lab";
import { useState } from "react";

import RestoreIcon from '@mui/icons-material/Restore';
import SimulationControlDrawer from "./SimulationControl";
import SimulationControl from "./SimulationControl";

const SideMenu = () => {
    const [value, setValue] = useState(0);

    const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
        setValue(newValue);
    };

    return (
        <>
            <TabContext value={value}>
                <TabList onChange={handleChange}>
                    <Tab label="Tab 1" value="1" />
                    <Tab label="Tab 2" value="2" />
                </TabList>

                <TabPanel value="1">
                    <SimulationControl />
                </TabPanel>
                <TabPanel value="2">
                    <h1> tab2 desu</h1>
                </TabPanel>
            </TabContext>
        </>
    )
};

export default SideMenu; 