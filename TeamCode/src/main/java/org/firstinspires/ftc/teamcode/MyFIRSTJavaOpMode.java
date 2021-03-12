package org.firstinspires.ftc.teamcode;

import android.text.style.TabStopSpan;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;

@Deprecated
@Disabled
@TeleOp(name="MyFIRSTJavaOpMode", group="default")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private static double[] MOTORCALIBRATION = {1, 1, 1, 1}; //0.82, 1, 0.88, 0.92
    private static double IMUFACTOR = 0;

    private DcMotor[] motors = new DcMotor[4];
    //private int[] encPos0 = new int[3];
    //private int[] encPos1 = new int[3];
    private double[] encDelta = new double[3]; //Encoder delta in radians.
    private Gamepad user1;
    private double[] wheelPowers;
    //private Vector2 pos = new Vector2(0, 0); //Position in meters
    //private double rot = 0; //Rotation in radians
    private Pose2d pose = new Pose2d(0, 0, 0);
    //private StandardTrackingWheelLocalizer odometry = new StandardTrackingWheelLocalizer(hardwareMap);

    @Override
    public void runOpMode() {
        int i;
        user1 = this.gamepad1;
        for(i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }
        //0: Front-Left, Negative is Forward. Right-side dead wheel encoder: Negative is forward.
        //1: Front-Right, Positive is Forward. Left-side dead wheel encoder: Negative is forward.
        //2: Back-Right, Positive is Forward. Rear dead wheel encoder: Negative is right.
        //3: Back-Left, Negative is Forward.

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //pos.set(0, 0);
        //rot = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Mecanum drive train
            wheelPowers = MyMathLib.mecMath(user1.left_stick_x, user1.left_stick_y, (user1.left_trigger - user1.right_trigger));
            for(i = 0; i < 4; i++) {
                motors[i].setPower(wheelPowers[i] * MOTORCALIBRATION[i]);
            }

            //Encoder velocities
            /*
            for(i = 0; i < 3; i++) {
                encPos0[i] = motors[i].getCurrentPosition();
                //encPos1[i] = motors[i].getCurrentPosition();
                //encDelta[i] = ((encPos1[i] - encPos0[i]) / ENCODERCYCLES) * 2 * Math.PI;
            }
             */

            /*
            odometry.setPos(encPos0);
            odometry.update();
            pose = odometry.getPoseEstimate();
             */

            //Odometry (old)
            //rot += MyMathLib.odometry(pos, rot, encDelta);

            //Odometry Reset (old)
            //if(user1.start) {
            //    pos.set(0, 0);
            //    rot = 0;
            //}

            /* Used this to identify motor locations.
            if(user1.a) {
                motors[0].setPower(0.25);
            }
            if(user1.b) {
                motors[1].setPower(0.25);
            }
            if(user1.x) {
                motors[2].setPower(0.25);
            }
            if(user1.y) {
                motors[3].setPower(0.25);
            }
             */

            //telemetry.addData("Controller Input", user1.left_stick_x + ", " + user1.left_stick_y + ", " + (user1.right_trigger - user1.left_trigger));
            //telemetry.addData("Motor Output", wheelPowers[0] + ", " + wheelPowers[1] + ", " + wheelPowers[2] + ", " + wheelPowers[3]);
            //telemetry.addData("Encoder Position", encPos1[0]);
            //telemetry.addData("Encoder Previous Position", encPos0[0]);
            //telemetry.addData("Encoder Delta", encPos1[0] - encPos0[0]);

            //telemetry.addData("Status", "Running");
            //RobotLog.d("Encoders: " + encVel[0] + ", " + encVel[1] + ", " + encVel[2]);
            //telemetry.update();

            telemetry.addData("Encoders", encDelta[0] + ", " + encDelta[1] + ", " + encDelta[2]);
            telemetry.addData("Pose", pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());
            telemetry.update();
            RobotLog.d("Encoders: " + encDelta[0] + ", " + encDelta[1] + ", " + encDelta[2]);
            RobotLog.d("Pose: " + pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());

            //encPos0 = encPos1.clone();
        }
    }

    /* No difference in performance from running encoders on main thread. Abandoned. Might be a good idea for later.
    private class EncoderThread extends Thread {
        @Override
        public void run() {
            while (opModeIsActive()) {
                int i;
                time1 = clock.time();

                //Encoder velocities
                for(i = 0; i < 3; i++) {
                    encPos1[i] = motors[i].getCurrentPosition();
                }
                if(time1 - time0 > 0.0) {
                    for(i = 0; i < 3; i++) {
                        encVel[i] = (double) (encPos1[i] - encPos0[i]) / ENCODERCYCLES / (time1 - time0);
                    }
                }

                telemetry.addData("Delta Time", time1 - time0);
                telemetry.addData("Encoders", encVel[0] + ", " + encVel[1] + ", " + encVel[2]);
                telemetry.update();
                RobotLog.d("Delta Time: " + (time1 - time0));
                RobotLog.d("Encoders: " + encVel[0] + ", " + encVel[1] + ", " + encVel[2]);

                time0 = time1;
                encPos0 = encPos1.clone();
            }
        }
    }
    */
}