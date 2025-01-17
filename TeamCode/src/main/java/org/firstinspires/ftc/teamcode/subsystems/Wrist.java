package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.WristPosition;

public class Wrist extends SubSystem{

    public WristPosition wristState;
    public Servo wrist;
    private RobotHardware robot;
    private final double WRIST_DOWN = 1;
    private final double WRIST_NEUTRAL = 0.5;
    private final double WRIST_UP = 0;

    public Wrist(RobotHardware robot) {this.robot = robot;}


    @Override
    public void init() {
        wristState = WristPosition.Neutral;
        wrist = robot.wristServo;
        wrist.setPosition(WRIST_NEUTRAL);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_DOWN);
            wristState = WristPosition.Down;
            return false;
        }
    }
    public Action wristDown() {return new WristNeutral();}

    
    public class WristNeutral implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_NEUTRAL);
            wristState = WristPosition.Neutral;
            return false;
        }
    }
    public Action wristNeutral() {return new WristNeutral();}


    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_UP);
            wristState = WristPosition.Up;
            return false;
        }
    }
    public Action wristUp() {return new WristNeutral();}
}
