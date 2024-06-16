package org.firstinspires.ftc.teamcode.drivepp;

import static org.firstinspires.ftc.teamcode.drivepp.pathBuilder.calculateVectorMagnitude;
import static org.firstinspires.ftc.teamcode.drivepp.pathBuilder.injectWaypoints;
import static org.firstinspires.ftc.teamcode.drivepp.pathBuilder.pointSmoothing;
import static org.firstinspires.ftc.teamcode.drivepp.pathBuilder.processWaypoints;
import static org.firstinspires.ftc.teamcode.drivepp.pathCalculuation.*;
import static org.firstinspires.ftc.teamcode.drivepp.PURE_PURSUIT_CONSTANTS.*;

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

        waypoints = processWaypoints(waypoints, spacing, weightData, weightSmooth, tolerance);
        calculateCurvature(waypoints);
        calculateVelocity(waypoints, maxVelocity, maxAcceleration, k);

        telemetry.addLine("Waypoints Processed!");
        telemetry.update();
        while (!isStopRequested()) {

            }
        }
    }
}