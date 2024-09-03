package org.firstinspires.ftc.teamcode.util.geometry;

public class PoseVelocity {
    public Vector2D linearVelocity;
    public double angularVelocity;

    public PoseVelocity(Vector2D linearVelocity, double angularVelocity){
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
    }

    public PoseVelocity(double x, double y, double angularVelocity){
        this.linearVelocity = new Vector2D(x, y);
        this.angularVelocity = angularVelocity;
    }

    public Vector2D getLinearVelocity() {
        return linearVelocity;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getX() {
        return linearVelocity.x;
    }

    public double getY() {
        return linearVelocity.y;
    }

    public void setLinearVelocity(Vector2D linearVelocity) {
        this.linearVelocity = linearVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

}
