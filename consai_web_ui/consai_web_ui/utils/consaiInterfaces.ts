import ROSLIB from "roslib";

export const publishReplacement = (
    ros: ROSLIB.Ros,
    type: "ball" | "robot",
    team: "blue" | "yellow" | null,
    id: number | null,
    pos: Vector2D,
) => {

const publisher = new ROSLIB.Topic({
    ros: ros,
    name: "/replacement",
    messageType: "robocup_ssl_msgs/msg/Replacement",
});

const generateMessage = (): ROSLIB.Message | null => {
    if (type === "ball") {
    return new ROSLIB.Message({
        ball: [{ x: [pos.x], y: [pos.y] }],
        robots: [],
    });
    }

    if (team === null || id === null) return null;

    if (type === "robot") {
    return new ROSLIB.Message({
        ball: [],
        robots: [
        {
            x: pos.x,
            y: pos.y,
            id: id,
            yellowteam: team === "yellow",
        },
        ],
    });
    }

    return null;
};

const message = generateMessage();
if (message) {
    publisher.publish(message);
}
};
