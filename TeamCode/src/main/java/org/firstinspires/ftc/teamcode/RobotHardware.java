package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;

    // 2002-0180-0002
    public Servo clawServo = null;
    // 2000-0025-0003
    public Servo bucketServo = null;
    // 2000-0025-0002
    public Servo wristServo = null;
    //public CRServo intakeServo = null;

    // 5203-2402-0019
    public DcMotorEx liftMotor = null;
    //5204-08139 series, 3895.9 resolution, for other arm thing motor
    //5203 series, 384.5 ppr - encoder resolution
    public DcMotorEx slideMotor = null;
    public Encoder slideEncoder = null;
    public IMU imu = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        stopAndResetEncoder(liftMotor);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        //TODO: Add pidf like Lift above if needed.
        slideMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopAndResetEncoder(slideMotor);
        slideEncoder = new OverflowEncoder(new RawEncoder(RobotHardware.this.slideMotor));
        slideEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set the slide motor for driver control.
        if (myOpMode.getClass().getSimpleName().equals("DriverControl")) {
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // .setPwmRange() is intended to enable the full range of the servos.
        bucketServo = myOpMode.hardwareMap.get(Servo.class, "bucketServo");
//        ((ServoImplEx) bucketServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        bucketServo.setDirection(Servo.Direction.REVERSE);
//        bucketServo.scaleRange(0.05, 0.75);
//
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");
//        ((ServoImplEx) clawServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        clawServo.scaleRange(0, 0.55);
//
        wristServo = myOpMode.hardwareMap.get(Servo.class, "wristServo");
//        ((ServoImplEx) wristServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        wristServo.setDirection(Servo.Direction.REVERSE);
//        wristServo.scaleRange(0.15, 0.7);
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

    /**
     * Scale a joystick value to smooth it for motor control.
     * Square results in finer control a slow speeds, but less aggressive than
     * the cube version.
     */
    public static double ScaleMotorSquare(double joyStickPosition) {
        boolean positionIsNegative = joyStickPosition < 0;
        double result = joyStickPosition * joyStickPosition;
        return positionIsNegative ? -result : result;
    }

    /**
     * Scale a joystick value to smooth it for motor setting.
     * The cube results in finer control at slow speeds.
     */
    public static double ScaleMotorCube(double joyStickPosition, boolean slowMode) {
        double p = Math.pow(joyStickPosition, 3.0);
        if (slowMode) p = p/2;
        return p;
    }

    /**
     * Scale the joystick value to smooth it for motor settings.
     * This algorithm gives a bit more sensitivity than the ScaleMotorCube() method.
     */
    public static double ScaleMotorTan(double input) {
        return (input / 1.07) * (.62 * (Math.pow(input, 2)) + .45);
    }
}
