package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * @author TM
 */
@TeleOp(name="TeleOpMode", group="default")
public class TeleOpMode extends BaseOpMode {
    @Override
    public void tick() {
        //Sticks and triggers for movement
        if(gamepad1.left_bumper) {
            wheelPowers = seekLocation(transform, SHOOTING_T[shoti]);
        } else {
            wheelPowers = mecMath(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger));
            if (gamepad1.left_stick_button) {
                for (i = 0; i < 4; i++) {
                    wheelPowers[i] *= SLOWSPEED;
                }
            }
        }
        for (i = 0; i < 4; i++) {
            drive[i].setPower(wheelPowers[i] * MOTORCALIB[i]);
        }

        //Wobble Controls
        wobbleAim.setPower(gamepad1.right_stick_y);
        if(gamepad1.right_stick_x > 0.2) {
            wobbleHand.setPosition(WOBBLE_AIM_POSITIONS[1]);
        } else if(gamepad1.right_stick_x < 0.2) {
            wobbleHand.setPosition(WOBBLE_AIM_POSITIONS[0]);
        }

        //Launch Controls
        if(gamepad1.dpad_up) {
            launchIndex = Math.min(launchIndex + 1, 2);
        } else if(gamepad1.dpad_down) {
            launchIndex = Math.max(launchIndex - 1, 0);
        }
        updateLaunchAim();

        //Intake Controls
        if(gamepad1.y) {
            powerIntake();
        } else {
            flywheel.setPower(gamepad1.right_bumper ? 1 : 0);
            magazine.setPower(gamepad1.b ? 1 : 0);
        }

        //Flywheel Controls
        if(gamepad1.right_bumper) {
            flywheel.setPower(1);
        }

        //Magazine Controls
        if(gamepad1.a) {
            magazine.setPower(1);
        }

        //Start: Reset position. Back: Change goal position.
        if(gamepad1.start) {
            transform.set(0, 0, 0);
        }
        if(gamepad1.back && backdeb) {
            backdeb = false;
            shoti = (shoti + 1) % 3;
        } else if(!gamepad1.back) {
            backdeb = true;
        }

        updateTelemetry();
    }
}