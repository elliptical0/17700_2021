package org.firstinspires.ftc.teamcode;

public class Constants {
    /**
     * 0.0 to 1.0, multiplier applied to motor power.
     * Compensates for hardware differences in motor speeds.
     * Order: Front-left, front-right, rear-right, rear-left.
     */
    public static final double[] MOTORCALIB = {1, 1, 1, 1};

    /**
     * Multiplier applied to motor speeds when holding down the left stick.
     */
    public static final double SLOWSPEED = 0.35;

    /**
     * Deadwheel radius.
     */
    public static final double DEADWHEEL_RADIUS = 0.995370749373427;

    /**
     * Encoder ticks per revolution.
     */
    public static final double TICKS_PER_REV = 8192;

    /**
     * Distance between the two lateral dead wheels.
     */
    public static final double LATERAL_DISTANCE = 11.440378084618675;

    /**
     * Distance between the rear dead wheel and the center.
     */
    public static final double REAR_OFFSET = 8.9560151267944225;

    /**
     * Number of spins during LATERAL_DISTANCE and REAR_OFFSET calibration. Higher = more accurate.
     */
    public static final double CALIB_SPINS = 14;

    /**
     * Number of inches traveled during DEADWHEEL_RADIUS calculation. Higher = more accurate.
     */
    public static final double CALIB_DIST = 12;

    /**
     * Shooting transform relative to the robot's starting location, in inches.
     * 3 different shooting positions can be stored and used (change with the BACK button).
     */
    public static final Transform[] SHOOTING_T = {
            new Transform(0, 0, 0),
            new Transform(0, 0, 0),
            new Transform(0, 0, 0)
    };

    /**
     *
     */
    public static final double[] LAUNCH_AIM_POSITIONS = {0, -0.2, 0.2};

    /**
     *
     */
    public static final double[] WOBBLE_AIM_POSITIONS = {0.5, 0.65, 0.9};

    public static final double[] WOBBLE_HAND_POSITIONS = {0.15, 0.65};

    /**
     * Deadzones for when the robot is autonomously seeking a position.
     */
    public static final double DEADZONE_POS = 0.75;
    public static final double DEADZONE_ANGLE = Math.toRadians(5);
}