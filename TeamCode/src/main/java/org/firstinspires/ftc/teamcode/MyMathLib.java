package org.firstinspires.ftc.teamcode;

import java.util.*;

// Math reference: https://seamonsters-2605.github.io/archive/mecanum/

 public class MyMathLib {
     //deprecated
     private static double DEADWHEELRADIUS = 0.0254; //Radius of dead wheels in meters. (1 inch)
     private static double TRACKWIDTH = 0.3443; //0.349 Distance between the parallel dead wheels in meters.
     private static double FORWARDOFFSET = 0.229; //0.229 Distance between center of rotation and rear dead wheel in meters.

     private static double LATERAL_DISTANCE = 14;

    /**
     * Rotational power will take 100% priority over directional power.
     * Right-side motors are inverted.
     * @param x Joystick X-axis, -1 to 1
     * @param y Joystick Y-axis, -1 to 1
     * @param r Rotational magnitude, -1 to 1
     * @return Motor speeds.
     * 0: Front-left.
     * 1: Front-right.
     * 2: Back-right.
     * 3: Back-left.
     * @author TM
     */
     public static double[] mecMath(double x, double y, double r) {
         double[] wheelPowers = new double[4];
         //double mag = Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))); // Magnitude
         double mag = Math.max(-1.0, Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))); // Magnitude
         double theta = Math.atan2(y, -x);
         double powerA = Math.max(-1.0, Math.min(1.0, Math.sqrt(2) * Math.sin(theta - 0.25 * Math.PI))) * mag;
         double powerB = Math.max(-1.0, Math.min(1.0, Math.sqrt(2) * Math.sin(theta + 0.25 * Math.PI))) * mag;
         double dirMag = 1.0 - Math.abs(r); // Directional Magnitude
         double rotMag = r; // Rotational Magnitude

         wheelPowers[0] = (rotMag + powerB * dirMag); // Front-left
         wheelPowers[1] = (rotMag - powerA * dirMag); // Front-right
         wheelPowers[2] = (rotMag - powerB * dirMag); // Back-right
         wheelPowers[3] = (rotMag + powerA * dirMag); // Back-left

         return wheelPowers;
     }

     /**
      * Updates the position of the robot based on the current velocity of the dead wheels, and returns the rotation delta.
      * @param pos
      * @param rot
      * @param encVel
      * @return rotation delta to be added to rot.
      */
     @Deprecated
     public static double odometry(Vector2 pos, double rot, double[] encVel) {
         Vector2 dv = new Vector2(
                 DEADWHEELRADIUS * 0.5 * (encVel[0] + encVel[1]),
                 DEADWHEELRADIUS * ((FORWARDOFFSET / TRACKWIDTH) * (encVel[0] - encVel[1]) + encVel[2]));
         double dr = (DEADWHEELRADIUS / TRACKWIDTH) * (encVel[1] - encVel[0]);
         dv.rotate(rot + (0.5 * dr));
         pos.add(dv);
         return dr;
     }

     public static double updatePos(Vector2 pos0, double theta, double[] wheelDelta) {
         double ds = (wheelDelta[0] + wheelDelta[1]) / 2;
         double dt = (wheelDelta[0] - wheelDelta[1]) / LATERAL_DISTANCE;
         return 0;
     }

     public static double getLateralDistance(double dt, double[] wheelPos) {
         return (wheelPos[0] - wheelPos[1]) / dt;
     }
 }