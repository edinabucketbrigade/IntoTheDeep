package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ClawPosition;

import java.util.concurrent.TimeUnit;

public class Claw extends SubSystem {

    public ClawPosition clawState;
    public Servo claw;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final double CLAW_OPEN = 0.975;
    private final double CLAW_CLOSED = 0.4;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double beginTime = -1.0;
    private double runTime = 0.0;
    // Time to move servo in milliseconds
    private final double MOVE_TIME = 750;

    public Claw(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        clawState = ClawPosition.Close;
        claw = robot.clawServo;
        claw.setPosition(CLAW_OPEN);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (clawState) {
            case Close:
                if (leftBumperPressed) {
                    claw.setPosition(CLAW_CLOSED);
                    clawState = ClawPosition.Open;
                }
            case Open:
                if (rightBumperPressed) {
                    claw.setPosition(CLAW_OPEN);
                    clawState = ClawPosition.Close;
                }
        }
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beginTime < 0) { // first time to run
                claw.setPosition(CLAW_OPEN);
                clawState = ClawPosition.Close;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            packet.put("bucket", claw.getPosition());
            packet.put("bucket timer", runTime);

            if (MOVE_TIME < runTime) {
                return true;
            } else {
                beginTime = -1;
                runTime = 0;
                return false;
            }
        }
    }

    public Action clawClose() {
        return new ClawClose();
    }

    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beginTime < 0) { // first time to run
                claw.setPosition(CLAW_CLOSED);
                clawState = ClawPosition.Open;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            packet.put("bucket", claw.getPosition());
            packet.put("bucket timer", runTime);

            if (MOVE_TIME < runTime) {
                return true;
            } else {
                beginTime = -1;
                runTime = 0;
                return false;
            }
        }
    }

    public Action clawOpen() {
        return new ClawOpen();
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}