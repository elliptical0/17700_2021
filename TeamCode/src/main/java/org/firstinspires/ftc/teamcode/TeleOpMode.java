package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.MyMathLib.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * @author TM
 */
@TeleOp(name="TeleOpMode", group="default")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        int i;
        double d;

        DcMotor[] motors = new DcMotor[4];
        double[] wheelPowers;

        double[] encoderPos = new double[3];
        double[] encoderDelta = new double[3];
        Vector2 position = new Vector2(0,0);
        double heading = 0;

        int shotpos = 0;
        boolean backdeb = true;

        for(i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //Left Stick: Movement. Left Stick Button: Slower movement.
            if(gamepad1.left_bumper) {
                wheelPowers = seekLocation(position, heading, SHOOTING_POS[shotpos], 0);
            } else {
                wheelPowers = mecMath(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger));
                if (gamepad1.left_stick_button) {
                    for (i = 0; i < 4; i++) {
                        wheelPowers[i] *= SLOWSPEED;
                    }
                }
            }
            for (i = 0; i < 4; i++) {
                motors[i].setPower(wheelPowers[i] * MOTORCALIB[i]);
            }


            //Odometry
            for(i = 0; i < 3; i ++) {
                d = -motors[i].getCurrentPosition(); //All encoders are reversed.
                encoderDelta[i] = d - encoderPos[i];
                encoderPos[i] = d;
            }
            heading += updatePosition(position, heading, encoderDelta);

            //Start: Reset position. Back: Change goal position.
            if(gamepad1.start) {
                position.set(0, 0);
                heading = 0;
            }
            if(gamepad1.back && backdeb) {
                backdeb = false;
                shotpos = (shotpos + 1) % 3;
            } else if(!gamepad1.back) {
                backdeb = true;
            }

            //Telemetry
            telemetry.addData("Position, Heading", position + ", " + Math.toDegrees(heading));
            telemetry.addData("", "");
            telemetry.addData("To calibrate", "drag the robot forward " + CALIB_DIST + " inches (and do nothing else) and read the value below.");
            telemetry.addData("DEADWHEEL_RADIUS (" + CALIB_DIST + ")", CALIB_DIST / (2 * Math.PI * ((encoderPos[0] + encoderPos[1]) / 2) / TICKS_PER_REV));
            telemetry.addData("To calibrate", "spin the robot in place counterclockwise " + CALIB_SPINS + " times (and do nothing else) and read the values below. (Depends upon DEADWHEEL_RADIUS)");
            telemetry.addData("LATERAL_DISTANCE (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[0]) - encoderToInch(encoderPos[1])) / (2 * CALIB_SPINS * Math.PI));
            telemetry.addData("REAR_OFFSET (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[2]) / (2 * CALIB_SPINS * Math.PI)));
            telemetry.update();
        }
    }
}