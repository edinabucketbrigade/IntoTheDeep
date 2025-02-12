package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.WristPosition;

import java.util.concurrent.TimeUnit;

public class Wrist extends SubSystem {

    public WristPosition wristState;
    public Servo wrist;
    private final RobotHardware robot;
    private Rotate rotate;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    public boolean DPAD_RIGHT = false;
    private final double WRIST_UP = 0.35;
    private final double WRIST_NEUTRAL = 0.75;
    private final double WRIST_DOWN = 0.85;
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private double beginTime = -1.0;
    private double runTime = 0.0;
    // Time to move servo in milliseconds
    private final double MOVE_TIME = 750;

    public Wrist(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        wristState = WristPosition.Down;
        wrist = robot.wristServo;
        wrist.setPosition(WRIST_UP);
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
                    wristState = WristPosition.Down;
                    break;
                }

                if (DPAD_RIGHT) {
                    wrist.setPosition(WRIST_NEUTRAL);
                    wristState = WristPosition.Neutral;
                    break;
                }
            case Neutral:
                if (DPAD_DOWN) {
                    wrist.setPosition(WRIST_DOWN);
                    wristState = WristPosition.Down;
                    break;
                }

                if (DPAD_UP) {
                    wrist.setPosition(WRIST_UP);
                    wristState = WristPosition.Up;
                    break;
                }
            default:
//                wrist.setPosition(WRIST_UP);
//                wristState=WristPosition.Up;
        }
        //rotate.update();
    }

    public void setRotate(Rotate rotate) {
        this.rotate = rotate;
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beginTime < 0) { // first time to run
                wrist.setPosition(WRIST_UP);
                wristState = WristPosition.Down;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            packet.put("bucket", wrist.getPosition());
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

    public Action wristDown() {
        return new WristDown();
    }


    public class WristNeutral implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beginTime < 0) { // first time to run
                wrist.setPosition(WRIST_NEUTRAL);
                wristState = WristPosition.Neutral;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            packet.put("bucket", wrist.getPosition());
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

    public Action wristNeutral() {
        return new WristNeutral();
    }


    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beginTime < 0) { // first time to run
                wrist.setPosition(WRIST_DOWN);
                wristState = WristPosition.Up;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            packet.put("bucket", wrist.getPosition());
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

    public Action wristUp() {
        return new WristUp();
    }

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
//        if (DPAD_DOWN && DPAD_RIGHT) {
//            rotate.setProperties(false, true);
//        } else {
//            rotate.setProperties(true, false);
//        }
    }
}
