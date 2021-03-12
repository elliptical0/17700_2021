package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.DriveConstants.*;

public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private static Pose2d[] DEADWHEELS = {new Pose2d(0, LATERAL_DISTANCE / 2, 0), //Left Dead Wheel position relative to center, in inches
            new Pose2d(0, -LATERAL_DISTANCE / 2, 0), //Right, x = -0.0625
            new Pose2d(-FORWARD_OFFSET, 0, Math.PI / 2)}; //Rear, y = -0.25
    private MotorEncoder[] encoders = new MotorEncoder[3];
    private Double[] wheelPos = new Double[3];
    private Double[] wheelVel = new Double[3];
    private int i;

    //Positions of encoders relative to center of rotation, in inches.
    public StandardTrackingWheelLocalizer(MyMecanumDrive myMecanumDrive) {
        super(Arrays.asList(DEADWHEELS));
        for(i = 0; i < 3; i++) {
            encoders[i] = new MotorEncoder(myMecanumDrive.motors.get((i*3)%4));
            encoders[i].setDirection(MotorEncoder.Direction.REVERSE);
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        for(i = 0; i < 3; i++) {
            wheelPos[i] = encoderTicksToInches(encoders[i].getCurrentPosition());
        }
        return Arrays.asList(wheelPos);
    }

    @NotNull
    @Override
    public List<Double> getWheelVelocities() {
        for(i = 0; i < 3; i++) {
            wheelVel[i] = encoderTicksToInches(encoders[i].getCorrectedVelocity());
        }
        return Arrays.asList(wheelVel);
    }
}