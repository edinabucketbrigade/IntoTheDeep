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

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

/*
 *  Converted to use RR.
 *  The code is structured as a LinearOpMode
 *
 *  The code REQUIRES that you have encoders on the drive motors.
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *
 *  Notes:
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 */

@Autonomous(name = "RR", group = "Robot")
//@Disabled
public class AutoRR extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);

    private double headingError = 0;
    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 2386;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.

    // A list of lists. Each item in paths defines a state in the autonomous process.
    public ArrayList<ArrayList> paths = new ArrayList<>();

    @Override
    public void runOpMode() {
        robot.init();
        Pose2d initialPose = new Pose2d(-24, -60, Math.tan(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Setup the paths.
        initializePaths();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        robot.imu.resetYaw();

        // Step through each segment of each path.
        for (ArrayList path : paths) {
            for (Object segment : path) {
            }
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Create segments and paths for the robot to follow.
     */
    public void initializePaths() {
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        // Segments are the parts of a path (one part of your autonomous strategy,)
    }
}
