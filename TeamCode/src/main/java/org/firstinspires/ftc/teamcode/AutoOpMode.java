package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

/**
 * @author TM
 */
@Autonomous(name="AutoOpMode", group="default")
public class AutoOpMode extends BaseOpMode {
    public int state = 0;

    public final double[] d = new double[3];

    public final Transform[] transforms = {new Transform(12, 0, 0),
    new Transform(12, 12, Math.toRadians(90)),
    new Transform(0, 12, Math.toRadians(0)),
    new Transform(12, 0, Math.toRadians(45))};

    public void moveState(int i) {
        if(seekLocation(transform, transforms[i]) == d) {
            state += 1;
        }
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
                moveState(2);
                break;
            case 3:
                moveState(3);
                break;
            case 4:
                stop();
                break;
        }
        updateTelemetry();
    }
}
