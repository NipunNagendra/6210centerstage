package org.firstinspires.ftc.teamcode.util.geometry;

public class Vector2D {
    public double x,y;

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2D fromHeadingAndMagnitude(double h, double m){
        return new Vector2D(Math.cos(h) * m, Math.sin(h) * m);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public Vector2D deadzoneX(double val){
        if(Math.abs(x) < val) return new Vector2D(0, y);
        return this;
    }

    public Vector2D mult(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D divide(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D subt(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D unit() {
        return this.divide(magnitude());
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(
                x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double cross(Vector2D other) {
        return x * other.y - y * other.x;
    }

    public Vector2D project(Vector2D other) {
        double magnitude = other.magnitude();
        double angle = angle();
        return new Vector2D(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    public double magnitudeSquared() {
        return x * x + y * y;
    }

    @Override
    public String toString() {
        return String.format("{%.2f, %.2f}", x, y);
    }

    public double atan() {
        return Math.atan2(y, x);
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }


    public double sqrNorm() {
        return x * x + y * y;
    }



    public static Vector2D polar(double r, double t) {
        return new Vector2D(r * Math.cos(t), r * Math.sin(t));
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public double getMagSq() {
        return x * x + y * y;
    }

    // Operations

    public Vector2D add(Vector2D other) {
        return add(this, other);
    }

    public Vector2D subtract(Vector2D other) {
        return subtract(this, other);
    }

    public Vector2D scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    public Vector2D scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    public static Vector2D add(Vector2D a, Vector2D b) {
        return new Vector2D(a.x + b.x, a.y + b.y);
    }

    public static Vector2D subtract(Vector2D a, Vector2D b) {
        return new Vector2D(a.x - b.x, a.y - b.y);
    }

    public static Vector2D scalarMultiply(Vector2D vec, double scalar) {
        return new Vector2D(vec.x * scalar, vec.y * scalar);
    }

    public static Vector2D scalarDivide(Vector2D vec, double scalar) {
        return new Vector2D(vec.x / scalar, vec.y / scalar);
    }

    public static Vector2D slerp(Vector2D a, Vector2D b, double t) {
        double aMag = a.getMagnitude();
        double aHead = a.getHeading();
        double bMag = b.getMagnitude();
        double bHead = b.getHeading();
        return polar((1 - t) * aMag + t * bMag, (1 - t) * aHead + t * bHead);
    }
    public double distanceTo(Vector2D other) {
        return Math.hypot(this.x - other.x, this.y - other.y);
    }

    public Vector2D normalize() {
        double magnitude = getMagnitude();
        if (magnitude == 0) {
            // Return a zero vector if the magnitude is zero to avoid division by zero
            return new Vector2D(0, 0);
        }
        return new Vector2D(x / magnitude, y / magnitude);
    }

    public Vector2D normal() {
        return new Vector2D(-y, x);
    }
}
