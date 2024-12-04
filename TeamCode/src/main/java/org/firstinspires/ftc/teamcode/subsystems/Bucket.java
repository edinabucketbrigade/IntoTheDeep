
package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.BucketPosition;

public class Bucket extends SubSystem {

    public BucketPosition bucketState;
    public Servo bucket;
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
        bucket = robot.bucketServo;
        bucketState = BucketPosition.Down;
        bucket.setPosition(BUCKET_DOWN);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (bucketState) {
            case Up:
                if (DPAD_DOWN) {
                    bucket.setPosition(BUCKET_DOWN);
                    bucketState = BucketPosition.Down;
                }
            case Down:
                if (DPAD_UP) {
                    bucket.setPosition(BUCKET_UP);
                    bucketState = BucketPosition.Up;
                }
        }
    }

    public class BucketDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucket.setPosition(BUCKET_DOWN);
            bucketState = BucketPosition.Down;
            return false;
        }
    }

    public Action bucketDown() {
        return new BucketDown();
    }

    public class BucketUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucket.setPosition(BUCKET_UP);
            bucketState = BucketPosition.Up;
            return false;
        }
    }

    public Action bucketUp() {
        return new BucketUp();
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        DPAD_DOWN = leftBumper;
        DPAD_UP = rightBumper;
    }
}