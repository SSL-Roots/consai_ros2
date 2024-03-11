import Head from "next/head";

import ROSLIB from "roslib";

import { useEffect, useState } from "react";

import { Rosconnection } from "@/components/RosConnection";

import dynamic from "next/dynamic";

import { FormControl, InputLabel, MenuItem, Select } from "@mui/material";

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
      <h1>Hello consai web ui</h1>

      <CurosrSelector />
      {/* <MsgBox ros={ros} /> */}
      <BirdView ros={ros} />
    </>
  );
}

const CurosrSelector = () => {
  const [age, setAge] = useState("");

  const handleChange = (event) => {
    setAge(event.target.value);
  };

  const ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
  const yellows = ids.map((id) => {
    return (
      <MenuItem key={id} value={`y${id}`}>
        Y{id}
      </MenuItem>
    );
  });

  const blues = ids.map((id) => {
    return (
      <MenuItem key={id} value={`b${id}`}>
        B{id}
      </MenuItem>
    );
  });

  return (
    <>
      <FormControl size="small">
        <InputLabel id="demo-simple-select-label">Cursor</InputLabel>
        <Select
          labelId="demo-simple-select-label"
          id="demo-simple-select"
          value={age}
          label="Age"
          onChange={handleChange}
        >
          <MenuItem value={"ball"}>Ball</MenuItem>
          {yellows}
          {blues}
        </Select>
      </FormControl>
      Age: {age}
    </>
  );
};

const MsgBox = ({ ros }) => {
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (!ros) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/chatter",
      messageType: "std_msgs/String",
    });

    listener.subscribe((message) => {
      // console.log(JSON.stringify(message));
      setMessage(message.data);
    });
  }, [ros]);

  return (
    <div>
      <h1>{message}</h1>
    </div>
  );
};
