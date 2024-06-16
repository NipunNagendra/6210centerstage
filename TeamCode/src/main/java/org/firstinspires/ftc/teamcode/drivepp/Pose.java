package org.firstinspires.ftc.teamcode.drivepp;

public class Pose {
    public double x;
    public double y;
    public double heading;

    public Pose(double x, double y, double heading) {
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
    public Pose add(Pose other) {
        return new Pose(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    public Pose sub(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    public Pose mul(double scalar) {
        return new Pose(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose div(double scalar) {
        return new Pose(this.x / scalar, this.y / scalar, this.heading / scalar);
    }

    public double dot(Pose other) {
        return this.x * other.x + this.y * other.y + this.heading * other.heading;
    }

    public double cross(Pose other) {
        return this.x * other.y - this.y * other.x;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.heading * this.heading);
    }

    public Pose normalize() {
        return this.div(this.mag());
    }

    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    public Pose rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Pose(this.x * cos - this.y * sin, this.x * sin + this.y * cos, this.heading);
    }

    public Pose project(Pose other) {
        return other.mul(this.dot(other) / other.mag());
    }
}
