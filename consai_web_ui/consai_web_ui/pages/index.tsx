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

const BirdView = dynamic(() => import("../components/birdview/birdview"), {
  ssr: false,
});

export default function Home() {
  const [ros, setRos] = useState(null);

  return (
    <>
      <Head>
        <title>CON-SAI Web UI</title>
      </Head>
      <Rosconnection port={9090} setRos={setRos} />

      <Box component="section">
        <Grid container spacing={2} alignItems="flex-start" justifyContent="center">
          <Grid item xs={9}>
            <BirdView ros={ros} />
          </Grid>
          <Grid item xs={3} alignItems="flex-start">
            <SideMenu />
          </Grid>
        </Grid>
      </Box>
    </>
  );
}
