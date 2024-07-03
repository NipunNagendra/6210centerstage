package org.firstinspires.ftc.teamcode.legacy;

public class Vector {
    public double x;
    public double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
    public Vector add(Vector other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }

    public Vector sub(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }

    public Vector mul(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }

    public Vector div(double scalar) {
        return new Vector(this.x / scalar, this.y / scalar);
    }

    public double dot(Vector other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vector other) {
        return this.x * other.y - this.y * other.x;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector normalize() {
        return this.div(this.mag());
    }

    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    public Vector rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }

    public Vector project(Vector other) {
        return other.mul(this.dot(other) / other.dot(other));
    }

    public Vector reflect(Vector normal) {
        return this.sub(normal.mul(2 * this.dot(normal)));
    }

    public Vector lerp(Vector other, double t) {
        return this.mul(1 - t).add(other.mul(t));
    }

    public Vector slerp(Vector other, double t) {
        double theta = Math.acos(this.dot(other) / (this.mag() * other.mag()));
        return this.mul(Math.sin((1 - t) * theta) / Math.sin(theta)).add(other.mul(Math.sin(t * theta) / Math.sin(theta)));
    }

    public Vector clone() {
        return new Vector(this.x, this.y);
    }

    public String toString() {
        return "X: " + this.x + ", Y: " + this.y;
    }

}
