package org.firstinspires.ftc.teamcode.legacy;

public class Poseold {
    public double x;
    public double y;
    public double heading;

    public Poseold(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
    public double getHeading(){
        return this.heading;
    }
    public Poseold add(Poseold other) {
        return new Poseold(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    public Poseold sub(Poseold other) {
        return new Poseold(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    public Poseold mul(double scalar) {
        return new Poseold(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Poseold div(double scalar) {
        return new Poseold(this.x / scalar, this.y / scalar, this.heading / scalar);
    }

    public double dot(Poseold other) {
        return this.x * other.x + this.y * other.y + this.heading * other.heading;
    }

    public double cross(Poseold other) {
        return this.x * other.y - this.y * other.x;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.heading * this.heading);
    }

    public Poseold normalize() {
        return this.div(this.mag());
    }

    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    public Poseold rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Poseold(this.x * cos - this.y * sin, this.x * sin + this.y * cos, this.heading);
    }

    public Poseold project(Poseold other) {
        return other.mul(this.dot(other) / other.mag());
    }
}
