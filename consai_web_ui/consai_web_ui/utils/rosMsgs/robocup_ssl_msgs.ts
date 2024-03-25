export type Replacement = {
    ball?: BallReplacement
    robots: RobotReplacement[]
}

type BallReplacement = {
    x?: number
    y?: number
    vx?: number
    vy?: number
}


type RobotReplacement = {
    x: number
    y: number
    dir: number
    id: number
    yellowteam: boolean
    turnon?: boolean[]
}