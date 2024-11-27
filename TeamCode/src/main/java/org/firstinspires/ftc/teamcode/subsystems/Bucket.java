
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.BucketPosition;

public class Bucket extends SubSystem {

    public BucketPosition bucketState;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    private final double BUCKET_DOWN = 1;
    private final double BUCKET_UP = 0.3;

    public Bucket(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        bucketState = BucketPosition.Down;
        robot.bucketServo.setPosition(BUCKET_DOWN);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (bucketState) {
            case Up:
                if (DPAD_DOWN) {
                    robot.bucketServo.setPosition(BUCKET_DOWN);
                    bucketState = BucketPosition.Down;
                }
            case Down:
                if (DPAD_UP) {
                    robot.bucketServo.setPosition(BUCKET_UP);
                    bucketState = BucketPosition.Up;
                }
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        DPAD_DOWN = leftBumper;
        DPAD_UP = rightBumper;
    }
}