
package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.RotatePosition;

public class Rotate extends SubSystem {

    public RotatePosition rotateState;
    public Servo rotate;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    private final double ROTATE_OUT = 1;
    private final double ROTATE_IN = 0.25;

    public Rotate(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        rotate = robot.rotateServo;
        rotateState = RotatePosition.Out;
        rotate.setPosition(ROTATE_OUT);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (rotateState) {
            case Out:
                if (DPAD_DOWN) {
                    rotate.setPosition(ROTATE_IN);
                    rotateState = RotatePosition.In;
                }
            case In:
                if (DPAD_UP) {
                    rotate.setPosition(ROTATE_OUT);
                    rotateState = RotatePosition.Out;
                }
        }
    }

    public class BucketDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_OUT);
            rotateState = RotatePosition.Out;
            return false;
        }
    }

    public Action bucketDown() {
        return new BucketDown();
    }

    public class BucketUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_IN);
            rotateState = RotatePosition.In;
            return false;
        }
    }

    public Action bucketUp() {
        return new BucketUp();
    }

    public void setProperties(boolean dpadDown, boolean dpadUp) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
    }
}