package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

public class AutoOpMode extends BaseOpMode {
    public int stage = 0;

    public final double[] d = new double[3];

    public final Transform[] transforms = {new Transform(12, 0, 0),
    new Transform(12, 12, Math.toRadians(90)),
    new Transform(0, 12, Math.toRadians(0)),
    new Transform(12, 0, Math.toRadians(45))};

    public void moveStage(int i) {
        if(seekLocation(transform, transforms[i]) == d) {
            stage += 1;
        }
    }

    @Override
    public void tick() {
        switch(stage) {
            case 0:
                moveStage(0);
                break;
            case 1:
                moveStage(1);
                break;
            case 2:
                moveStage(2);
                break;
            case 3:
                moveStage(3);
                break;
            case 4:
                stop();
                break;
        }
        updateTelemetry();
    }
}
