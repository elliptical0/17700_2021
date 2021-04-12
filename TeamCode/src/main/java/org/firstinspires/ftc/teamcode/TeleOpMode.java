package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * @author TM
 */
@TeleOp(name="TeleOpMode", group="default")
public class TeleOpMode extends BaseOpMode {
    private boolean backdeb = true;
    private boolean rstickdeb = true;
    private boolean paddeb = true;

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

        //Start: Reset position. Back: Change goal position.
        if(gamepad1.start) {
            resetPosition();
        }
        if(gamepad1.back && backdeb) {
            backdeb = false;
            shoti = (shoti + 1) % 3;
        } else if(!gamepad1.back) {
            backdeb = true;
        }

        //Ends the loop here if the servos aren't active.
        if(!SERVOS_ACTIVE) {
            updateTelemetry();
            return;
        }

        //Intake Controls
        powerIntake(gamepad1.b);

        //Wobble Controls
        if(gamepad1.right_stick_y < -0.2 && rstickdeb) {
            if(wobbleAim.getPosition() == WOBBLE_AIM_POSITIONS[1]) {
                wobbleAimIndex = 2;
            } else {
                wobbleAimIndex = 1;
            }
            rstickdeb = false;
        } else if(gamepad1.right_stick_y > 0.2 && rstickdeb) {
            if(wobbleAim.getPosition() == WOBBLE_AIM_POSITIONS[2]) {
                wobbleAimIndex = 1;
            } else {
                wobbleAimIndex = 0;
            }
            rstickdeb = false;
        } else if(gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {
            rstickdeb = true;
        }
        updateWobbleAim();

        if(gamepad1.right_stick_x > 0.2) {
            wobbleHandIndex = 1;
        } else if(gamepad1.right_stick_x < -0.2) {
            wobbleHandIndex = 0;
        }
        updateWobbleHand();

        //Launch Controls
        if(gamepad1.dpad_up && paddeb) {
            paddeb = false;
            launchIndex = Math.min(launchIndex + 1, LAUNCH_AIM_POSITIONS.length - 1);
        } else if(gamepad1.dpad_down && paddeb) {
            paddeb = false;
            launchIndex = Math.max(launchIndex - 1, 0);
        } else if(!(gamepad1.dpad_up || gamepad1.dpad_down)) {
            paddeb = true;
        }
        updateLaunchAim();

        //Flywheel Controls
        if(gamepad1.right_bumper) {
            flywheel.setPower(-1);
        }

        //Magazine Controls
        if(gamepad1.a) {
            magazine.setPower(1);
        }

        updateTelemetry();
    }
}