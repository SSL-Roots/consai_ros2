'use client'

import Image from "next/image";
import styles from "./page.module.css";
import { useState } from "react";

import { Rosconnection } from "@/app/lib/RosConnection";

export default function Home() {
  const [ros, setRos] = useState(null);

  return (
    <>
      <Rosconnection rosUrl="ws://127.0.0.1:9090" setRos={setRos} />
      <h1>Hello consai web ui</h1>
    </>
  );
}
