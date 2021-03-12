package org.firstinspires.ftc.teamcode;

/**
 * Two-dimensional vector.
 * @author TM
 */
public class Vector2 {
    double x;
    double y;

    public Vector2(double x_, double y_) {
        x = x_;
        y = y_;
    }

    /**
     * Creates a unit vector in direction theta
     * @param theta in radians
     */
    public Vector2(double theta) {
        x = Math.cos(theta);
        y = Math.sin(theta);
    }

    public void set(double x_, double y_) {
        x = x_;
        y = y_;
    }

    public boolean equals(Vector2 vector) {
        return((vector.x == x) && (vector.y == y));
    }
    public boolean equals(double x_, double y_) {
        return((x_ == x) && (y_ == y));
    }

    public void add(Vector2 vector) {
        x += vector.x;
        y += vector.y;
    }
    public void add(double x_, double y_) {
        x += x_;
        y += y_;
    }

    public void subtract(Vector2 vector) {
        x -= vector.x;
        y -= vector.y;
    }
    public void subtract(double x_, double y_) {
        x -= x_;
        y -= y_;
    }

    public void multiply(Vector2 vector) {
        x *= vector.x;
        y *= vector.y;
    }
    public void multiply(double x_, double y_) {
        x *= x_;
        y *= y_;
    }

    public void divide(Vector2 vector) {
        x /= vector.x;
        y /= vector.y;
    }
    public void divide(double x_, double y_) {
        x /= x_;
        y /= y_;
    }

    public double length() {
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    /**
     * Rotate counter-clockwise.
     * @param theta in radians.
     */
    public void rotate(double theta) {
        set(Math.cos(theta) * x - Math.sin(theta) * y, Math.sin(theta) * x + Math.cos(theta) * y);
    }
    /**
     * Rotate clockwise.
     * @param theta in radians.
     */
    public void rotateClock(double theta) {
        rotate(-theta);
    }

    /**
     * @return in radians.
     */
    public double direction() {
        return Math.atan2(y, x);
    }
}