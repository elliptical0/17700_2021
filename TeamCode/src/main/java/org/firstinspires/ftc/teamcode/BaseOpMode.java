package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * The base op mode, contains useful variables and debug stuff which is useful in auto and teleop.
 * @author TM
 */
public class BaseOpMode extends LinearOpMode {
    int i;
    double d;

    ElapsedTime clock = new ElapsedTime();
    double currentTime;
    double deltaTime;

    DcMotor[] drive = new DcMotor[4];
    DcMotor[] motors = new DcMotor[3];
    DcMotor intake;
    DcMotor flywheel;
    CRServo magazine;
    Servo wobbleAim;
    Servo wobbleHand;
    Servo[] launchAim = new Servo[2]; //Must be in unison
    double[] launchAimStart = new double[2];
    int launchIndex = 0;

    double[] wheelPowers;

    double[] encoderPos = new double[3];
    double[] encoderDelta = new double[3];
    Transform transform = new Transform(0, 0, 0);

    int shoti = 0;

    public void tick() {
        updateTelemetry();
    }

    public void updateTime() {
        deltaTime = currentTime - clock.seconds();
        currentTime = clock.seconds();
    }

    public void updateOdometry() {
        for(i = 0; i < 3; i ++) {
            d = -drive[i].getCurrentPosition(); //All encoders are reversed.
            encoderDelta[i] = d - encoderPos[i];
            encoderPos[i] = d;
        }
        updatePosition(transform, encoderDelta);
    }

    public void updateTelemetry() {
        telemetry.addData("Transform", transform);
        telemetry.addData("", "");
        telemetry.addData("To calibrate", "drag the robot forward " + CALIB_DIST + " inches (and do nothing else) and read the value below.");
        telemetry.addData("DEADWHEEL_RADIUS (" + CALIB_DIST + ")", CALIB_DIST / (2 * Math.PI * ((encoderPos[0] + encoderPos[1]) / 2) / TICKS_PER_REV));
        telemetry.addData("To calibrate", "spin the robot in place counterclockwise " + CALIB_SPINS + " times (and do nothing else) and read the values below. (Depends upon DEADWHEEL_RADIUS)");
        telemetry.addData("LATERAL_DISTANCE (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[0]) - encoderToInch(encoderPos[1])) / (2 * CALIB_SPINS * Math.PI));
        telemetry.addData("REAR_OFFSET (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[2]) / (2 * CALIB_SPINS * Math.PI)));
        telemetry.update();
    }

    public void powerIntake(boolean on) {
        if(on) {
            intake.setPower(1);
            magazine.setPower(-1);
            flywheel.setPower(-1);
        } else {
            intake.setPower(0);
            magazine.setPower(0);
            flywheel.setPower(0);
        }
    }

    public void updateLaunchAim() {
        for(i = 0; i < 2; i++) {
            launchAim[i].setPosition(launchAimStart[i] + LAUNCH_AIM_POSITIONS[launchIndex]);
        }
    }

    public void runOpMode() {
        for (i = 0; i < 4; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }
        intake = hardwareMap.get(DcMotor.class, "motor6");
        flywheel = hardwareMap.get(DcMotor.class, "motor5");
        magazine = hardwareMap.get(CRServo.class, "servo5");
        wobbleAim = hardwareMap.get(Servo.class, "servo4");
        wobbleHand = hardwareMap.get(Servo.class, "servo3");
        for(i = 0; i < 2; i++) {
            launchAim[i] = hardwareMap.get(Servo.class, "servo" + i);
            launchAim[i].setDirection(i == 0 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            launchAimStart[i] = launchAim[i].getPosition();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            updateTime();
            updateOdometry();
            tick();
        }
    }
}
