package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

// Math reference: https://seamonsters-2605.github.io/archive/mecanum/

 public class MyMathLib {
     /**
      * @author TM
      * @param v
      * @param min
      * @param max
      * @return
      */
     public static double clamp(double v, double min, double max) {
         return Math.max(min, Math.min(max, v));
     }

     /**
      * @author TM
      * @param v
      * @param min
      * @return
      */
     public static double absMin(double v, double min) {
         if(Math.abs(v) < min) {
             if(v >= 0) {
                 return min;
             } else {
                 return -min;
             }
         } else {
             return v;
         }
     }

     /**
      * @author TM
      * @param t0 theta
      * @return Smallest co-terminal angle
      */
     public static double coterminal(double t0) {
         double t1 = t0 % (Math.PI * 2);
         if(t1 > Math.PI) {
             t1 -= (Math.PI * 2);
         }
         return t1;
     }

     /**
      * Turn controller input into motor output for a Mecanum drive
      * @author TM
      * @param x Joystick X-axis, -1 to 1
      * @param y Joystick Y-axis, -1 to 1
      * @param r Rotational magnitude, -1 to 1
      * Rotational power will take 100% priority over directional power.
      * Note: Right-side motors are inverted. If this code does not work, change the inversion before anything else.
      */
     public static double[] mecMath(double x, double y, double r) {
         double[] wheelPowers = new double[4];
         //double mag = Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))); // Magnitude
         double mag = Math.max(-1.0, Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))); // Magnitude
         double theta = Math.atan2(y, -x);
         double powerA = clamp(Math.sqrt(2) * Math.sin(theta - 0.25 * Math.PI), -1, 1) * mag;
         double powerB = clamp(Math.sqrt(2) * Math.sin(theta + 0.25 * Math.PI), -1, 1) * mag;
         double dirMag = 1.0 - Math.abs(r); // Directional Magnitude
         double rotMag = r; // Rotational Magnitude

         wheelPowers[0] = (rotMag + powerB * dirMag); // Front-left
         wheelPowers[1] = (rotMag - powerA * dirMag); // Front-right
         wheelPowers[2] = (rotMag - powerB * dirMag); // Back-right
         wheelPowers[3] = (rotMag + powerA * dirMag); // Back-left

         return wheelPowers;
     }

     /**
      * @author TM
      * @param pos0 Current position
      * @param head0 Current heading
      * @param pos1 Target position
      * @param head1 Target heading
      * @return Wheel powers to drive toward the target position.
      */
     public static double[] seekLocation(Vector2 pos0, double head0, Vector2 pos1, double head1) {
         Vector2 v = new Vector2(0, 0);
         double r = 0;
         Vector2 dir = pos1.subtract(pos0);
         if(dir.length() > DEADZONE_POS) {
             v = dir.rotate(coterminal(head0)).divide(12, 12);
         }
         if(Math.abs(head1 - coterminal(head0)) > DEADZONE_ANGLE) {
             r = absMin(clamp(head1 - coterminal(head0), -1, 1), 0.2);
         }
         return mecMath(v.y, -v.x, r);
     }

     /**
      * @author TM
      * @param ticks
      * @return inches
      */
     public static double encoderToInch(double ticks) {
         return DEADWHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
     }

    /**
     * Update estimated position using the dead wheel encoders
     * @author TM
     * @param pos Previous position
     * @param head Previous heading
     * @param denc Delta of encoders
     * @return Change in heading
     * This is the odometry updating function.
     */
     public static double updatePosition(Vector2 pos, double head, double[] denc) {
         double[] dwheel = new double[3];
         for(int i = 0; i < 3; i++) {
             dwheel[i] = encoderToInch(denc[i]);
         }
         double dt = (dwheel[0] - dwheel[1]) / LATERAL_DISTANCE;
         double df = (dwheel[0] + dwheel[1]) / 2;
         double dr = (dwheel[2]) - dt * REAR_OFFSET;
         Vector2 dpos = new Vector2(df * Math.cos(head + dt / 2), df * Math.sin(head + dt / 2));
         dpos.set(dpos.add(dr * -Math.sin(head + dt / 2), dr * Math.cos(head + dt / 2)));
         pos.set(pos.add(dpos));
         return dt;
     }
 }