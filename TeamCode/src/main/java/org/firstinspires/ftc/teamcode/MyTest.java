package org.firstinspires.ftc.teamcode;

import java.util.*;

 public class MyTest {
     public static void main(String[] args) {
         double x = -1;
         double y = -1;
         double z = 0.0;
         double[] result = MyMathLib.mecMath(x, y, z);
         System.out.println(result[0]);
         System.out.println(result[1]);
         System.out.println(result[2]);
         System.out.println(result[3]);
     }
 }