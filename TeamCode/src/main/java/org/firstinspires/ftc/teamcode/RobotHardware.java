package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public Servo clawServo = null;
    public Servo bucketServo = null;

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
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        clawServo = myOpMode.hardwareMap.get(Servo.class,"clawServo");
        bucketServo = myOpMode.hardwareMap.get(Servo.class,"bucketServo");

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

        if (max > 1) {
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
}
