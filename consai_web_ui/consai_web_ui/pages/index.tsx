import Head from "next/head";
import Image from "next/image";
import { Inter } from "next/font/google";
import styles from "@/styles/Home.module.css";

import ROSLIB from "roslib";

import { useEffect, useState } from "react";

import { Rosconnection } from "@/components/RosConnection";

import dynamic from "next/dynamic";

const inter = Inter({ subsets: ["latin"] });

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
      {/* <MsgBox ros={ros} /> */}
      <BirdView ros={ros} />
    </>
  );
}

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
