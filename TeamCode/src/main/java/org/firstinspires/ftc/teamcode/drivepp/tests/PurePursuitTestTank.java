package org.firstinspires.ftc.teamcode.drivepp.tests;

import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_TRANSLATIONAL_SPEED;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.drivepp.PurePursuitPath;
import org.firstinspires.ftc.teamcode.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.drivepp.Waypoint;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class PurePursuitTestTank extends OpMode {
    private Drivetrain drivetrain;
    private TwoWheelIMULocalizerLegacy localizer;
    private PurePursuitPath purePursuitPath;
    private FtcDashboard dashboard;
    private Pose endPose;
    private List<Point> robotPath;
    private List<Point> targetPath;

    private boolean PID = false;
    private boolean finished = false;

    public static CustomBasicPID distanceController = new CustomBasicPID(new PIDCoefficients(XPIDTest.xP, 0, XPIDTest.xD));
    public static CustomBasicPID angleController = new CustomBasicPID(new PIDCoefficients(HeadingPIDTest.hP, 0, HeadingPIDTest.hD));

    private ElapsedTime accelLimit;
    private final double ACCEL_LIMIT = 0.5;

    private ElapsedTime timer;

    public static double lookaheadDistance = 15;
    public static double threshold = 2.0; // Threshold for considering the distance "close enough"
    public static double angleThreshold = 0.1; // Threshold for considering the angle "close enough"

    private boolean loopIsActive = true;

    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        localizer.setPose(-45.5, -39.5, Math.PI);
        purePursuitPath = new PurePursuitPath(
                new Waypoint(new Point(-45.5, -39.5), lookaheadDistance),
                new Waypoint(new Point(-58, -24), lookaheadDistance),
                new Waypoint(new Point(-58, 29), lookaheadDistance),
                new Waypoint(new Pose(-38, 40, -0.282), lookaheadDistance)
        );
        endPose = purePursuitPath.getEndPose();

        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(endPose.x, endPose.y, 9);
        fieldOverlay.strokeLine(endPose.x, endPose.y, endPose.x + Math.cos(endPose.heading) * 9, endPose.y + Math.sin(endPose.heading) * 9);

        for (Waypoint waypoint : purePursuitPath.getWaypoints()) {
            fieldOverlay.strokeCircle(waypoint.getPoint().x, waypoint.getPoint().y, 2);
        }
        dashboard.sendTelemetryPacket(packet);

        robotPath = new ArrayList<>();
        targetPath = new ArrayList<>();
    }

    public void loop() {
        if (!loopIsActive) return;

        localizer.update();
        execute();
        telemetry.update();

        Pose robotPose2 = localizer.getPose();
        robotPath.add(new Point(robotPose2.x, robotPose2.y));
    }

    public void execute() {
        if (accelLimit == null) accelLimit = new ElapsedTime();
        if (purePursuitPath.isFinished()) PID = true;

        Pose robotPose = localizer.getPose();
        Pose targetPose = purePursuitPath.update(robotPose);
        targetPath.add(new Point(targetPose.x, targetPose.y));
        if (PID && timer == null) {
            timer = new ElapsedTime();
        }

        double xError = targetPose.x - robotPose.x;
        double yError = targetPose.y - robotPose.y;
        double distance = Math.hypot(xError, yError);
        double forwardPower, turnPower;

        forwardPower = distanceController.calculate(0, distance);
        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetPose.heading - currentHeading);
        turnPower = angleController.calculate(0, -error);

//        forwardPower *= Math.cos(Range.clip(AngleUnit.normalizeRadians(targetPose.heading-robotPose.heading), -Math.PI / 2, Math.PI / 2));

        forwardPower = Range.clip(forwardPower, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        turnPower = Range.clip(turnPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        forwardPower *= Math.cos(Range.clip(error, -Math.PI/2, Math.PI/2));

        drivetrain.setTankPower(-forwardPower, -turnPower);

        sendTelemetry(robotPose, targetPose);
    }

    public boolean isFinished() {
        return finished || (timer != null && timer.milliseconds() > 2000);
    }

    public void end(boolean interrupted) {
        drivetrain.setFieldWeightedDrivePower(new Pose(), 0);
    }

    private void sendTelemetry(Pose robotPose, Pose targetPose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(targetPose.x, targetPose.y, 9);
        fieldOverlay.strokeLine(targetPose.x, targetPose.y, targetPose.x + Math.cos(targetPose.heading) * 9, targetPose.y + Math.sin(targetPose.heading) * 9);

        for (int i = 0; i < targetPath.size() - 1; i++) {
            Point p1 = targetPath.get(i);
            Point p2 = targetPath.get(i + 1);
            fieldOverlay.strokeLine(p1.x, p1.y, p2.x, p2.y);
        }

        fieldOverlay.setStroke("#00FF00");
        fieldOverlay.strokeCircle(robotPose.x, robotPose.y, 9);
        fieldOverlay.strokeLine(robotPose.x, robotPose.y, robotPose.x + Math.cos(robotPose.heading) * 9, robotPose.y + Math.sin(robotPose.heading) * 9);

        for (int i = 0; i < robotPath.size() - 1; i++) {
            Point p1 = robotPath.get(i);
            Point p2 = robotPath.get(i + 1);
            fieldOverlay.strokeLine(p1.x, p1.y, p2.x, p2.y);
        }

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(endPose.x, endPose.y, 9);
        fieldOverlay.strokeLine(endPose.x, endPose.y, endPose.x + Math.cos(endPose.heading) * 9, endPose.y + Math.sin(endPose.heading) * 9);

        for (Waypoint waypoint : purePursuitPath.getWaypoints()) {
            fieldOverlay.strokeCircle(waypoint.getPoint().x, waypoint.getPoint().y, 2);
        }

        packet.put("target Heading", targetPose.heading);
        packet.put("current Heading", robotPose.heading);
        dashboard.sendTelemetryPacket(packet);
    }
}
