/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of poses.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 */

@Autonomous(name = "Drive By Gyro", group = "Robot")
//@Disabled
public class AutoGyro extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    public RobotHardware robot = new RobotHardware(this);

    /**
     * A location (pose) for use in creating paths.
     */
    public static class Pose {
        double x;
        double y;
        double heading;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public List<Pose> poses = new ArrayList<Pose>();


    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        robot.init();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        robot.imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup the paths.
        initializePoses();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.imu.resetYaw();

        // Step through each leg of the path,
        for (Pose pose : poses) {
            moveRobot(pose.x, pose.y, pose.heading);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Create a list of poses for the robot to follow.
     */
    public void initializePoses() {
        poses.add(new Pose(10, 20, 0));
    }
}
