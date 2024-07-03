package org.firstinspires.ftc.teamcode.legacy;

import static org.firstinspires.ftc.teamcode.legacy.pathBuilder.processWaypoints;
import static org.firstinspires.ftc.teamcode.legacy.pathCalculuation.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class pptest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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

        telemetry.addLine("Waypoints Processed!");
        telemetry.update();
        while (!isStopRequested()) {

            }
        }
    }
}