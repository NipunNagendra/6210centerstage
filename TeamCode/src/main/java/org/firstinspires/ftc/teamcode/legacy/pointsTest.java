package org.firstinspires.ftc.teamcode.legacy;

import static org.firstinspires.ftc.teamcode.legacy.pathBuilder.processWaypoints;
import static org.firstinspires.ftc.teamcode.legacy.pathCalculuation.*;

public class pointsTest {
    public static void main(String[] args) {
        Waypoint[] waypoints = {
                new Waypoint(0, 0),
                new Waypoint(1, 1),
                new Waypoint(2, 0),
                new Waypoint(3, 1),
                new Waypoint(4, 0)
        };
        double maxVelocity = 50;

        waypoints = processWaypoints(waypoints, PURE_PURSUIT_CONSTANTS.spacing, PURE_PURSUIT_CONSTANTS.weightData, PURE_PURSUIT_CONSTANTS.weightSmooth, PURE_PURSUIT_CONSTANTS.tolerance);
        calculateCurvature(waypoints);
        calculateVelocity(waypoints, maxVelocity, PURE_PURSUIT_CONSTANTS.maxAcceleration, PURE_PURSUIT_CONSTANTS.k);


        for (Waypoint wp : waypoints) {
            System.out.println("X: " + wp.x + ", Y: " + wp.y + ", Curvature: " + wp.curvature + ", Velocity: " + wp.velocity);
        }
    }
}
