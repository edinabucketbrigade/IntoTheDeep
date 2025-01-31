package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.WristPosition;

public class Wrist extends SubSystem {

    public WristPosition wristState;
    public Servo wrist;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    public boolean DPAD_RIGHT = false;
    private final double WRIST_DOWN = 0.35;
    private final double WRIST_NEUTRAL = 0.5;
    private final double WRIST_UP = 0.9;

    public Wrist(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        wristState = WristPosition.Down;
        wrist = robot.wristServo;
        wrist.setPosition(WRIST_DOWN);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        switch (wristState) {
            case Down:
                if (DPAD_UP) {
                    wrist.setPosition(WRIST_UP);
                    wristState = WristPosition.Up;
                    break;
                }

                if (DPAD_RIGHT) {
                    wrist.setPosition(WRIST_NEUTRAL);
                    wristState = WristPosition.Neutral;
                    break;
                }
            case Up:
                if (DPAD_DOWN) {
                    wrist.setPosition(WRIST_DOWN);
                    wristState=WristPosition.Down;
                    break;
                }

                if(DPAD_RIGHT){
                    wrist.setPosition(WRIST_NEUTRAL);
                    wristState=WristPosition.Neutral;
                    break;
                }
            case Neutral:
                if (DPAD_DOWN) {
                    wrist.setPosition(WRIST_DOWN);
                    wristState=WristPosition.Down;
                    break;
                }

                if(DPAD_UP){
                    wrist.setPosition(WRIST_UP);
                    wristState=WristPosition.Up;
                    break;
                }
            default:
//                wrist.setPosition(WRIST_UP);
//                wristState=WristPosition.Up;
        }
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_DOWN);
            wristState = WristPosition.Down;
            return false;
        }
    }

    public Action wristDown() {
        return new WristDown();
    }


    public class WristNeutral implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_NEUTRAL);
            wristState = WristPosition.Neutral;
            return false;
        }
    }

    public Action wristNeutral() {
        return new WristNeutral();
    }


    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_UP);
            wristState = WristPosition.Up;
            return false;
        }
    }

    public Action wristUp() {
        return new WristUp();
    }

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
    }
}
