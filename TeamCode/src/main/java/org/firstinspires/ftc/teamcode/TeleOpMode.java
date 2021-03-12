package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MyMathLib;

import java.util.List;

@TeleOp(name="TeleOpMode", group="default")
public class TeleOpMode extends LinearOpMode {
    public static final double SLOW_MULTIPLIER = 0.35; //Motor speed reduction from holding the L-Stick button
    //DcMotor[] motors = new DcMotor[4];
    //0: Front-Left, Negative is (not) Forward. Right-side dead wheel encoder: Negative is forward.
    //1: Front-Right, Positive is Forward. Left-side dead wheel encoder: Negative is forward.
    //2: Back-Right, Positive is Forward. Rear dead wheel encoder: Negative is right.
    //3: Back-Left, Negative is (not) Forward.

    @Override
    public void runOpMode() {
        int i;
        MyMecanumDrive drive = new MyMecanumDrive(hardwareMap);
        double[] wheelPowers;
        /*
        for(i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }
         */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            wheelPowers = MyMathLib.mecMath(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger));
            if(gamepad1.left_stick_button) {
                for(i = 0; i < 4; i++) {
                    wheelPowers[i] *= SLOW_MULTIPLIER;
                }
            }
            drive.setMotorPowers(wheelPowers[0], wheelPowers[3], wheelPowers[2], wheelPowers[1]);

            drive.update();
            drive.updatePoseEstimate();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            List<Double> wheelPositions = ((StandardTrackingWheelLocalizer) drive.localizer).getWheelPositions();
            telemetry.addData("wheelPositions", wheelPositions.get(0) + ", " + wheelPositions.get(1) + ", " + wheelPositions.get(2));
            telemetry.update();
        }
    }
}