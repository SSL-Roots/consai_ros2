'use client'

import React, { useEffect } from 'react';
import ROSLIB from 'roslib';


export const Rosconnection = ({ port, rosDomainId, setRos}: { port: number, rosDomainId: string, setRos: any}) => {

  useEffect(() => {
    const generateHost = (port: number) => {
      const hostname = `${window.location.hostname}`;
      if (hostname.includes("github.dev")) {
        // Github Codespaces上で動作しているとき、ホスト名は次のような形になる
        // xxx-yyy-zzz-<port>.app.github.dev
        // ここから<port>の部分をport propsの値に置換して返す
        const parts = hostname.split("-");
        return parts[0] + "-" + parts[1] + "-" + parts[2] + "-" + port + ".app.github.dev";
      }
      
      return hostname + ":" + port;
    };

    const hostname = generateHost(port);
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const rosUrl = `${protocol}//${hostname}`;

    console.log(`Connecting to ${rosUrl}`)

    const ros = new ROSLIB.Ros({
      url: rosUrl,
      options: {
        ros_domain_id: rosDomainId // ROS_DOMAIN_IDを設定する
      }
    });

    ros.on("connection", () => {
      setRos(ros);
      console.log('Connected to ROSBridge WebSocket server.');
    });
  
    ros.on('error', function(error) {
      console.log('Error connecting to ROSBridge WebSocket server: ', error);
    });
  
    ros.on('close', function() {
      console.log('Connection to ROSBridge WebSocket server closed.');
    });

    return () => {
      ros.close();
    };
  }, [port, rosDomainId, setRos]);

  return (
    <>
      <h1>ROS Connection</h1>
    </>
  );
}