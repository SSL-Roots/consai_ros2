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
        <Grid container spacing={2} alignItems="center" justifyContent="center">
          <Grid item xs={12}>
            <BirdView ros={ros} />
          </Grid>
        </Grid>
      </Box>
    </>
  );
}
