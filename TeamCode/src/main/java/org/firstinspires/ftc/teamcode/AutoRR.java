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

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.enums.StartPosition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.ArrayList;
import java.util.List;

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

@Autonomous(name = "RR", group = "Robot", preselectTeleOp = "DriverControl")
//@Disabled
public class AutoRR extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);
    private MecanumDrive drive;
    private final Lift lift = new Lift(robot);
    private final Bucket bucket = new Bucket(robot);
    private final Claw claw = new Claw(robot);
    private final Arm arm = new Arm(robot);
    private Pose2d initialPose;
    // Trajectories and Actions for RR to follow.
    private StartPosition startPosition = StartPosition.None;
    private TrajectoryActionBuilder moveToBuckets;
    private Action moveFromBucketsToObservatory;

    @Override
    public void runOpMode() {
        robot.init();
        //TODO: Add menu code here to get start position
        if (startPosition == StartPosition.Left) {
            initialPose = new Pose2d(-24, -60, Math.tan(0));
        }

        if (startPosition == StartPosition.Right) {
            initialPose = new Pose2d(12, -60, Math.tan(0));
        }

        drive = new MecanumDrive(hardwareMap, initialPose);
        // Setup the paths.
        initializePath();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Make sure the imu is correct.
        robot.imu.resetYaw();

        Actions.runBlocking(
                new SequentialAction(
                        moveToBuckets.build(),
                        lift.lifHigh(),
                        new SleepAction(.5),
                        bucket.bucketUp(),
                        bucket.bucketDown(),
                        lift.liftDown(),
                        moveFromBucketsToObservatory
                ));

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
     * Create RR Trajectories and Actions.
     */
    public void initializePath() {
        moveToBuckets = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));
        //
        moveFromBucketsToObservatory = moveToBuckets.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -60), Math.toRadians(0))
                .build();
    }
}
