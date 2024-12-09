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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

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
    // Poses for key locations on the field.
    private Vector2d bucketDropPosition = new Vector2d(-50, -50);
    private Pose2d bucketDropPose = new Pose2d(bucketDropPosition, Math.toRadians(45));
    private Vector2d observatoryPosition = new Vector2d(50, -60);
    private Pose2d observatoryPose = new Pose2d(observatoryPosition, Math.toRadians(0));
    // Trajectories and Actions for RR to follow.
    private Action moveToBuckets;
    private Action moveFromBucketsToObservatory;

    @Override
    public void runOpMode() {
        robot.init();
        lift.init();
        bucket.init();
        AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
        autonomousConfiguration.init(this.gamepad1, this.telemetry, hardwareMap.appContext);

        // Loop until menu selections are made.
        while (!opModeIsActive()) {
            autonomousConfiguration.init_loop();
        }

        if (autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Left) {
            initialPose = new Pose2d(24, 60, Math.toRadians(0));
        }

        if (autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {
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

        // Delay if requested.
        sleep(autonomousConfiguration.getDelayStartSeconds());

        Actions.runBlocking(
                new SequentialAction(
                        moveToBuckets,
                        lift.lifHigh(),
                        new SleepAction(.5),
                        bucket.bucketUp(),
                        // Wait for bucket to dump.
                        new SleepAction(1),
                        bucket.bucketDown(),
                        // Make sure bucket is down.
                        new SleepAction(1),
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
                .strafeToLinearHeading(bucketDropPosition, bucketDropPose.heading)
                .build();
        // After bucket drop drive back to observatory.
        moveFromBucketsToObservatory = drive.actionBuilder(bucketDropPose)
                .strafeToLinearHeading(observatoryPosition, observatoryPose.heading)
                .build();
    }
}
