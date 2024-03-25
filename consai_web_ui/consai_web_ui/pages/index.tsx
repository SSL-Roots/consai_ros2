import Head from "next/head";

import ROSLIB from "roslib";

import { useEffect, useState } from "react";

import { Rosconnection } from "@/components/RosConnection";

import dynamic from "next/dynamic";

import {
  Box,
  Container,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
} from "@mui/material";
import { Grid } from "@mui/material";
import SideMenu from "@/components/sideMenu/SideMenu";
import { BirdViewMouseEvent } from "../components/birdview/birdview";
import SimulationControl from "@/components/sideMenu/SimulationControl";

const BirdView = dynamic(() => import("../components/birdview/birdview"), {
  ssr: false,
});

export default function Home() {
  const [ros, setRos] = useState(null);
  const [mouseEvent, setMouseEvent] = useState<BirdViewMouseEvent | null>(null);

  return (
    <>
      <Head>
        <title>CON-SAI Web UI</title>
      </Head>
      <Rosconnection port={9090} setRos={setRos} />

      <Box component="section">
        <Grid container spacing={2} alignItems="flex-start" justifyContent="center">
          <Grid item xs={10}>
            <BirdView ros={ros} setMouseEvent={setMouseEvent} />
          </Grid>
          <Grid item xs={2} alignItems="flex-start">
            <SideMenu
              children={[
                {
                  label: "Simulation Control",
                  value: "simulation_control",
                  component: < SimulationControl ros={ros} mouseEvent={mouseEvent} />
                }
              ]} />
          </Grid>
        </Grid>
      </Box>
    </>
  );
}
