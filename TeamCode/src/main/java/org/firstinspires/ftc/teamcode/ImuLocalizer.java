package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ImuLocalizer {
    double conversionFactor = 162.15, prevTime = 0;
    double curPoseY = 0;
    double curPoseX = 0;
    ElapsedTime driveTime = new ElapsedTime();
    RobotHardware robot;

    public ImuLocalizer(RobotHardware robot) {
        this.robot = robot;
    }

    // https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
    public void updatePositionEncoderOdoBackup() {
        // apply mecnaum kinematic model (with wheel velocities [ticks per sec])
        double xV = (robot.leftFrontDrive.getVelocity() + robot.rightFrontDrive.getVelocity()
                + robot.leftBackDrive.getVelocity() + robot.rightBackDrive.getVelocity()) * 0.5;

        double yV = (-robot.leftFrontDrive.getVelocity() + robot.rightFrontDrive.getVelocity()
                + robot.leftBackDrive.getVelocity() - robot.rightBackDrive.getVelocity()) * 0.5;

        //TODO: figure our getAngle()
        // rotate the vector
        double nx = (xV * Math.cos(Math.toRadians(getAngle()))) - (yV * Math.sin(Math.toRadians(getAngle())));
        double nY = (xV * Math.sin(Math.toRadians(getAngle()))) + (yV * Math.cos(Math.toRadians(getAngle())));
        xV = nx;
        yV = nY;

        // integrate velocity over time
        curPoseY += (yV * (driveTime.seconds() - prevTime)) / conversionFactor; // <-- Tick to inch conversion factor
        curPoseX += (xV * (driveTime.seconds() - prevTime)) / conversionFactor;
        prevTime = driveTime.seconds();
    }
}
