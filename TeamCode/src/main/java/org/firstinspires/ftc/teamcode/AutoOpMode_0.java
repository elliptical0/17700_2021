package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

/**
 * @author TM
 */
@Autonomous(name="AutoOpMode_0", group="default")
public class AutoOpMode_0 extends BaseOpMode {
    public int state = 0;

    public final double[] D = new double[4];

    public final int STARTING_T_I = 0;

    double stateStartTime;
    int starterStackPosition;

    public void changeState(int i) {
        state = i;
        stateStartTime = currentTime;
    }

    public final Transform[] transforms = {new Transform(12, 12, 0),
            new Transform(72, 12, Math.toRadians(45)),
            new Transform(97.5, 28, Math.toRadians(0)),
            new Transform(24, 12, Math.toRadians(180)),
            new Transform(60, 12, Math.toRadians(180))};

    public void moveState(int i) {
        moveState(i, state + 1);
    }

    public void moveState(int i, int nextState) {
        if(seekLocation(transform, transforms[i]) == D) {
            changeState(nextState);
        }
    }

    public void examineStarterStack() {
        starterStackPosition = 0;
        changeState(state + 1);
    }

    @Override
    public void initialize() {
        SharedVariables.startingTransform(STARTING_T_I);
        super.initialize();
    }

    @Override
    public void tick() {
        switch(state) {
            case 0:
                moveState(0);
                break;
            case 1:
                moveState(1);
                break;
            case 2:
                examineStarterStack();
                break;
            case 3:
                moveState(2);
                break;
            case 4:
                wobbleAimIndex = 2;
                wobbleHandIndex = 1;
                if(currentTime - stateStartTime > 2) {
                    changeState(5);
                }
                break;
            case 5:
                wobbleHandIndex = 0;
                if(currentTime - stateStartTime > 0.25) {
                    wobbleAimIndex = 1;
                }
                if(currentTime - stateStartTime > 1.25) {
                    switch(starterStackPosition) {
                        case 0:
                            changeState(6);
                            break;
                    }
                }
                break;
            case 6:
                moveState(3, 10);
                break;
            case 10:
                wobbleAimIndex = 2;
                if(currentTime - stateStartTime > 1) {
                    changeState(11);
                }
                break;
            case 11:
                wobbleHandIndex = 0;
                if(currentTime - stateStartTime > 0.25) {
                    wobbleAimIndex = 0;
                }
                if(currentTime - stateStartTime > 1.25) {
                    changeState(12);
                }
                break;
            case 12:
                moveState(4, 99);
                break;
            case 99:
                stop();
                break;
        }

        updateTelemetry();
    }
}
