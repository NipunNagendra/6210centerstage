package org.firstinspires.ftc.teamcode.drivepp.tests;

import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.X_GAIN;

import static java.lang.Thread.sleep;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.drivepp.PurePursuitPath;
import org.firstinspires.ftc.teamcode.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.drivepp.Waypoint;
import org.firstinspires.ftc.teamcode.util.TelemetryDrawer;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;

import java.util.ArrayList;
import java.util.List;

@Config

@Autonomous
public class PurePursuitTest extends OpMode {
    private Drivetrain drivetrain;
    private TwoWheelIMULocalizerLegacy localizer;
    private PurePursuitPath purePursuitPath;
    private FtcDashboard dashboard;
    private Pose endPose;
    private List<Point> robotPath;
    private List<Point> targetPath;

    private boolean PID = false;
    private boolean finished = false;

    public static CustomBasicPID xController = new CustomBasicPID(new PIDCoefficients(XPIDTest.xP, 0, XPIDTest.xD));
    public static CustomBasicPID yController = new CustomBasicPID(new PIDCoefficients(YPIDTest.yP, 0, YPIDTest.yD));
    public static CustomBasicPID hController = new CustomBasicPID(new PIDCoefficients(HeadingPIDTest.hP, 0, HeadingPIDTest.hD));

    private ElapsedTime accelLimit;
    private final double ACCEL_LIMIT = 0.5;

    private ElapsedTime timer;

    public static double lookaheadDistance = 15;

    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        localizer.setPose(-45.5,-39.5,Math.PI);
        purePursuitPath = new PurePursuitPath(
                new Waypoint(new Point(-45.5,-39.5), lookaheadDistance),
                new Waypoint(new Point(-58, -24), lookaheadDistance),
                new Waypoint(new Point(-58, 29), lookaheadDistance),
                new Waypoint(new Pose(-38, 40, -0.282), lookaheadDistance)
        );
        endPose = purePursuitPath.getEndPose();

        dashboard = FtcDashboard.getInstance();
        robotPath = new ArrayList<>();
        targetPath = new ArrayList<>();

        TelemetryPacket packet = new TelemetryPacket();
        TelemetryDrawer drawer = new TelemetryDrawer(packet);

        drawer.drawPose(endPose, "#0000FF", 9);
        drawer.drawWaypoints(purePursuitPath, "#0000FF", 2);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
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

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConstants.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConstants.ALLOWED_HEADING_ERROR) finished = true;

        double currentX = robotPose.x;
        double xPower = xController.calculate(targetPose.x, currentX);

        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetPose.heading - currentHeading);
        double hPower = hController.calculate(0, -error);

        double currentY = robotPose.y;
        double yPower = yController.calculate(targetPose.y, currentY);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        TelemetryPacket packet = new TelemetryPacket();
        TelemetryDrawer drawer = new TelemetryDrawer(packet);

        drawer.drawPose(targetPose, "#FF0000", 9);
        drawer.drawPath(targetPath, "#FF0000");
        drawer.drawPose(robotPose, "#00FF00", 9);
        drawer.drawPath(robotPath, "#00FF00");
        drawer.drawPose(endPose, "#0000FF", 9);
        drawer.drawWaypoints(purePursuitPath, "#0000FF", 2);

        packet.put("target Heading", targetPose.heading);
        packet.put("current Heading", currentHeading);
        packet.put("heading power", hPower);
        dashboard.sendTelemetryPacket(packet);

        drivetrain.setFieldWeightedDrivePower(new Pose(x_rotated, y_rotated, hPower), 0);
    }

    public boolean isFinished() {
        return PID && finished || (timer != null && timer.milliseconds() > 2000);
    }

    public void end(boolean interrupted) {
        drivetrain.setFieldWeightedDrivePower(new Pose(), 0);
    }
}