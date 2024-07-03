package org.firstinspires.ftc.teamcode.legacy;

public class Waypoint {
    double x;
    double y;
    double distance;
    double curvature;
    double velocity;
    boolean endpoint = false;

    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Waypoint(Waypoint waypoint) {
        this.x = waypoint.x;
        this.y = waypoint.y;
    }

    public void setDistance(double distance){
        this.distance = distance;
    }
    public void setCurvature(double curvature){
        this.curvature = curvature;
    }
    public void setEndpoint(boolean endpoint){this.endpoint = endpoint;}
}
