
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.BucketPosition;

public class Bucket extends SubSystem {

    public BucketPosition bucketState;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final int BUCKET_DOWN = 0;
    private final int BUCKET_UP = 90;

    public Bucket(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        bucketState = BucketPosition.Down;
        robot.bucketServo.setPosition(0);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (bucketState) {
            case Up:
                if (leftBumperPressed) {
                    robot.bucketServo.setPosition(BUCKET_DOWN);
                    bucketState = BucketPosition.Down;
                }
            case Down:
                if (rightBumperPressed) {
                    robot.bucketServo.setPosition(BUCKET_UP);
                    bucketState = BucketPosition.Up;
                }
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}