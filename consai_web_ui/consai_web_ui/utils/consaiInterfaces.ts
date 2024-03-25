import ROSLIB from "roslib";
import { Replacement } from "./rosMsgs/robocup_ssl_msgs";

const createReplacementPublisher = (ros: ROSLIB.Ros) => {
    return new ROSLIB.Topic({
        ros: ros,
        name: "/replacement",
        messageType: "robocup_ssl_msgs/msg/Replacement",
    });
}

export const publishReplacement = (
    ros: ROSLIB.Ros,
    type: "ball" | "robot",
    team: "blue" | "yellow" | null,
    id: number | null,
    pos: Vector2D,
) => {
    const publisher = createReplacementPublisher(ros);

    const generateMessage = (): ROSLIB.Message | null => {
        if (type === "ball") {
        return new ROSLIB.Message({
            ball: [{ x: [pos.x], y: [pos.y] }],
            robots: [],
        } as Replacement);
        }

        if (team === null || id === null) return null;

        if (type === "robot") {
        return new ROSLIB.Message({
            robots: [
            {
                x: pos.x,
                y: pos.y,
                id: id,
                yellowteam: team === "yellow",
            },
            ],
        } as Replacement);
        }

        return null;
    };

    const message = generateMessage();
    console.log(message)
    if (message) {
        publisher.publish(message);
    }
};


export const publishRobotsOff = (
    ros: ROSLIB.Ros,
    team: "blue" | "yellow",
    robotIds: number[],
) => {
    const publisher = createReplacementPublisher(ros);

    const generateMessage = (): ROSLIB.Message => {
        return new ROSLIB.Message({
            robots: robotIds.map((id) => {
                return {
                x: id * 0.3,
                y: team === "yellow" ? 10: -10,
                id: id,
                yellowteam: team === "yellow",
                turnon: [false],
                };
            }),
        } as Replacement);
    };

    const message = generateMessage();
    console.log(message)
    publisher.publish(message);
};

export const publishRobotsOn = (
    ros: ROSLIB.Ros,
    team: "blue" | "yellow",
    robotIds: number[],
) => {
    const publisher = createReplacementPublisher(ros);

    const generateMessage = (): ROSLIB.Message => {
        return new ROSLIB.Message({
            robots: robotIds.map((id) => {
                return {
                x: id * 0.3,
                y: team === "yellow" ? 4: -4,
                id: id,
                yellowteam: team === "yellow",
                turnon: [true],
                };
            }),
        } as Replacement);
    };


    const message = generateMessage();
    publisher.publish(message);
}
