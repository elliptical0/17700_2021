package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoCon {
    Servo servo;
    double currentPos;
    double speed = 0.5;

    public ServoCon(Servo servo) {
        this.servo = servo;
        currentPos = servo.getPosition();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setPower(double power, double deltaTime) {
        currentPos += power * speed * deltaTime;
        servo.setPosition(currentPos);
    }
}
