'use client'

import React, { useEffect } from 'react';
import ROSLIB from 'roslib';


export const Rosconnection = ({ rosUrl, rosDomainId, setRos}: { rosUrl: string, rosDomainId: string, setRos: any}) => {

  useEffect(() => {
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
  }, [rosUrl, rosDomainId, setRos]);

  return (
    <>
      <h1>ROS Connection</h1>
    </>
  );
}