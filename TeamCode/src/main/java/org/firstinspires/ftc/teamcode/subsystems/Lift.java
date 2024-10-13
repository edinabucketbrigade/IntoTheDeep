package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.enums.LiftPosition;

public class Lift extends SubSystem {
    LiftPosition liftPosition;

    @Override
    public void init() {
        liftPosition = LiftPosition.Down;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        switch (liftPosition){
            case Down:
            case LowBasket:
            case HighBasket:
        }
    }

    public void setProperties(boolean buttonA, boolean buttonX, boolean buttonY) {
        if (buttonA) {
            liftPosition = LiftPosition.Down;
        } else if (buttonX) {
            liftPosition = LiftPosition.LowBasket;
        } else if (buttonY) {
            liftPosition = LiftPosition.HighBasket;
        }
    }
}
