
package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.BucketPosition;

import java.util.concurrent.TimeUnit;

public class Bucket extends SubSystem {

    public BucketPosition bucketState;
    public Servo bucket;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    private final double BUCKET_DOWN = 1;
    private final double BUCKET_UP = 0.25;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double beginTime = -1.0;
    private double runTime = 0.0;
    // Time to move servo in milliseconds
    private final double MOVE_TIME = 750;

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
            if (beginTime < 0) { // first time to run
                bucket.setPosition(BUCKET_DOWN);
                bucketState = BucketPosition.Down;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            telemetryPacket.put("bucket", bucket.getPosition());
            telemetryPacket.put("bucket timer", runTime);

            if (MOVE_TIME < runTime) {
                return true;
            } else {
                beginTime = -1;
                runTime = 0;
                return false;
            }
        }
    }

    public Action bucketDown() {
        return new BucketDown();
    }

    public class BucketUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (beginTime < 0) { // first time to run
                bucket.setPosition(BUCKET_UP);
                bucketState = BucketPosition.Up;
                beginTime = elapsedTime.now(TimeUnit.MILLISECONDS); // record time we start running
            } else {
                runTime = elapsedTime.now(TimeUnit.MILLISECONDS) - beginTime; // how long have we been running
            }

            telemetryPacket.put("bucket", bucket.getPosition());
            telemetryPacket.put("bucket timer", runTime);

            if (MOVE_TIME < runTime) {
                return true;
            } else {
                beginTime = -1;
                runTime = 0;
                return false;
            }
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