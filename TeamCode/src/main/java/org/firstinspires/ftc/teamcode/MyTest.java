package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.*;

 public class MyTest {
     public static void main(String[] args) {
         for(int i = 0; i < 3; i++) {
             System.out.println(((i*3)%4));
         }
     }
 }