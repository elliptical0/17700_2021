package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

/**
 * @author TM
 */
@Autonomous(name="AutoOpMode_0", group="default")
public class AutoOpMode_0 extends BaseOpMode {
    public int state = 0;

    public final double[] D = {0, 0, 0, 0};

    public final int STARTING_T_I = 0;

    private VisionThread visionThread = new VisionThread();

    double stateStartTime;

    public void changeState(int i) {
        state = i;
        stateStartTime = currentTime;
    }

    public final Transform[] transforms = {new Transform(12, 12, 0),
            new Transform(72, 12, Math.toRadians(45)),
            new Transform(97.5, 28, Math.toRadians(0)),
            new Transform(72, 12, Math.toRadians(180)),
            new Transform(48, 36, Math.toRadians(180)),
            new Transform(24, 12, Math.toRadians(180)),
            new Transform(60, 12, Math.toRadians(180))
    };

    public void moveState(int i) {
        moveState(i, state + 1);
    }

    public void moveState(int i, int nextState) {
        wheelPowers = seekLocation(transform, transforms[i]);
        if(Math.abs(wheelPowers[0] + wheelPowers[1] + wheelPowers[2] + wheelPowers[3]) < 0.05) {
            changeState(nextState);
            telemetry.addData("Change!",0);
        }
    }

    public void examineStarterStack() {
        if(visionThread.stackSize == 4) {
            visionThread.interrupt();
        } else if(visionThread.stackSize == 1) {
            visionThread.interrupt();
        }
        changeState(state + 1);
    }

    @Override
    public void initialize() {
        SharedVariables.startingTransform(STARTING_T_I);
        super.initialize();
    }

    @Override
    public void tick() {
        wheelPowers = D;
        switch(state) {
            case 0:
                moveState(0);
                break;
            case 1:
                moveState(1);
                break;
            case 2:
                visionThread = new VisionThread();
                visionThread.start();
                changeState(3);
            case 3:
                examineStarterStack();
                if(currentTime - stateStartTime > 2 || visionThread.stackSize != 0) {
                    visionThread.interrupt();
                    changeState(4);
                }
                break;
            case 4:
                moveState(2);
                break;
            case 5:
                wobbleAimIndex = 2;
                wobbleHandIndex = 1;
                if(currentTime - stateStartTime > 2) {
                    changeState(6);
                }
                break;
            case 6:
                wobbleHandIndex = 0;
                if(currentTime - stateStartTime > 0.25) {
                    wobbleAimIndex = 1;
                }
                if(currentTime - stateStartTime > 1.25) {
                    switch(visionThread.stackSize) {
                        case 0:
                            changeState(7);
                        case 1:
                            changeState(8);
                            break;
                        case 4:
                            changeState(9);
                            break;
                    }
                }
                break;
            case 7:
                moveState(3, 10);
                break;
            case 8:
                moveState(4, 10);
            case 9:
                moveState(5, 10);
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
                moveState(6, 99);
                break;
            case 99:
                stop();
                break;
        }
        updateMotors();
        telemetry.addData("State:", state);
        telemetry.addData("Stack Size:", visionThread.stackSize);
        updateTelemetry();
    }
}
