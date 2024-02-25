'use client'

import Image from "next/image";
import styles from "./page.module.css";
import { useEffect, useState } from "react";

import { Rosconnection } from "@/app/lib/RosConnection";
import ROSLIB from "roslib";

export default function Home() {
  const [ros, setRos] = useState(null);

  return (
    <>
      <Rosconnection rosUrl="ws://127.0.0.1:9090" setRos={setRos} />
      <h1>Hello consai web ui</h1>
      <MsgBox ros={ros} />
    </>
  );
}


const MsgBox = ({ros}) => {
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (!ros) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/robot0/command",
      messageType: "consai_frootspi_msgs/msg/RobotCommand",
    });

    listener.subscribe((message) => {
      // console.log(JSON.stringify(message));
      setMessage(message.velocity_x);
    });
  }, [ros]);

  return (
    <div>
      <h1>{message}</h1>
    </div>
  );
}
