package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    //public Servo clawServo = null;
    public Servo bucketServo = null;
    public CRServo intakeServo = null;

    public DcMotorEx liftMotor = null;
    //5203 series, 384.5 ppr - encoder resolution
    //5204-08139 series, 3895.9 resolution, for other arm thing motor
    public DcMotorEx armMotor = null;
    public IMU imu = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        stopAndResetEncoder(liftMotor);
        PIDFCoefficients pidfVelocityCoefficients = liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfPositionCoefficients = liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        // These values are recommended as a starting point if you are tuning a PID.
        pidfVelocityCoefficients.p = 1.063f;
        pidfVelocityCoefficients.i = 1.063f;
        pidfVelocityCoefficients.f = 10.63f;
        liftMotor.setVelocityPIDFCoefficients(pidfVelocityCoefficients.p, pidfVelocityCoefficients.i, pidfVelocityCoefficients.d, pidfVelocityCoefficients.f);
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfPositionCoefficients);
        liftMotor.setPositionPIDFCoefficients(8f);
        liftMotor.setTargetPositionTolerance(10);

        armMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        //clawServo = myOpMode.hardwareMap.get(Servo.class,"clawServo");
        bucketServo = myOpMode.hardwareMap.get(Servo.class, "bucketServo");
        intakeServo = myOpMode.hardwareMap.get(CRServo.class, "intakeServo");

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
        /*
         This is a version from gm0.org.

            x *= 1.1 // Adjust for imperfect strafing.
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;
         */

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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * This seems to be the only way to reliably stop a motor and reset the encoder.
     * This wos only tested on a goBilda motor.
     *
     * @param motor Motor to stop.
     */
    public void stopAndResetEncoder(DcMotorEx motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
