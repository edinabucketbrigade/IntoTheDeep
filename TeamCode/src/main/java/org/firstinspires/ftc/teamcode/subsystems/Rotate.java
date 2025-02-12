
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
    public boolean RIGHT_BUMPER = false;
    public boolean LEFT_BUMPER = false;
    private final double ROTATE_OUT = 1;
    private final double ROTATE_IN = 0.375;

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
                if (LEFT_BUMPER) {
                    rotate.setPosition(ROTATE_IN);
                    rotateState = RotatePosition.In;
                }
            case In:
                if (RIGHT_BUMPER) {
                    rotate.setPosition(ROTATE_OUT);
                    rotateState = RotatePosition.Out;
                }
        }
    }

    public class RotateOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_OUT);
            rotateState = RotatePosition.Out;
            return false;
        }
    }

    public Action rotateOut() {
        return new RotateOut();
    }

    public class RotateIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_IN);
            rotateState = RotatePosition.In;
            return false;
        }
    }

    public Action rotateIn() {
        return new RotateIn();
    }

    public void setProperties(boolean rotateIn, boolean rotateOut) {
        this.LEFT_BUMPER = rotateIn;
        this.RIGHT_BUMPER = rotateOut;
    }
}