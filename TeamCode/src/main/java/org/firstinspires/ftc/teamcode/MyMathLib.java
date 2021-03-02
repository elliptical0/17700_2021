package org.firstinspires.ftc.teamcode;

import java.util.*;

// Math reference: https://seamonsters-2605.github.io/archive/mecanum/

/**
 * @author TM
 * x = Joystick X-axis, -1 to 1
 * y = Joystick Y-axis, -1 to 1
 * r = Rotational magnitude, -1 to 1
 * Rotational power will take 100% priority over directional power.
 * Note: Right-side motors are inverted. If this code does not work, change the inversion before anything else.
 */
 public class MyMathLib {
     public static double PI = 3.141592653589793; // 3.14159265358979323846
     public static double[] mecMath(double x, double y, double r){
         double[] wheelPowers = new double[4];
         //double mag = Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))); // Magnitude
         double mag = Math.max(-1.0, Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))); // Magnitude
         double theta = Math.atan2(y, -x);
         double powerA = Math.max(-1.0, Math.min(1.0, Math.sqrt(2) * Math.sin(theta - 0.25 * PI))) * mag;
         double powerB = Math.max(-1.0, Math.min(1.0, Math.sqrt(2) * Math.sin(theta + 0.25 * PI))) * mag;
         double dirMag = 1.0 - Math.abs(r); // Directional Magnitude
         double rotMag = r; // Rotational Magnitude

         wheelPowers[0] = (rotMag + powerB * dirMag); // Front-left
         wheelPowers[1] = (rotMag - powerA * dirMag); // Front-right
         wheelPowers[2] = (rotMag - powerB * dirMag); // Back-right
         wheelPowers[3] = (rotMag + powerA * dirMag); // Back-left

         return wheelPowers;
     }
 }