package org.firstinspires.ftc.teamcode.backend.drivepp.tests;

import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.HeadingPIDTest.hD;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.HeadingPIDTest.hI;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.HeadingPIDTest.hP;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.XPIDTest.xD;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.XPIDTest.xI;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.XPIDTest.xP;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.YPIDTest.yD;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.YPIDTest.yI;
import static org.firstinspires.ftc.teamcode.backend.drivepp.tests.YPIDTest.yP;

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
import org.firstinspires.ftc.teamcode.backend.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.backend.localizers.RawOtosLocalizer;
import org.firstinspires.ftc.teamcode.util.controllers.CustomBasicSQUID;
import org.firstinspires.ftc.teamcode.util.controllers.CustomBasicSQUIDOG;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

@Config
@Autonomous
public class P2PTest extends OpMode {
    Drivetrain drivetrain;
    RawOtosLocalizer localizer;
    private FtcDashboard dashboard;

    public static double targetX= 20;
    public static double targetY= 0;
    public static double targetH= 3.14;


    public Pose targetPose = new Pose(targetX, targetY, targetH);
    public Pose robotPose2 = new Pose(0, 0, 0);

    public static CustomBasicSQUIDOG xController = new CustomBasicSQUIDOG(new PIDCoefficients(xP, xI, xD));
    public static CustomBasicSQUID yController = new CustomBasicSQUID(new PIDCoefficients(yP, yI, yD));
    public static CustomBasicSQUID hController = new CustomBasicSQUID(new PIDCoefficients(hP, hI, hD));

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = 0.02;

    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double STABLE_MS = 100;
    public static double DEAD_MS = 2500;

    private final double  MAX_TRANSLATIONAL_SPEED = 0.5;
    private final double  MAX_ROTATIONAL_SPEED = 0.4;
    private final double X_GAIN = 2.00;

    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new RawOtosLocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetPose.x, targetPose.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(targetPose.x, targetPose.y, targetPose.x + Math.cos(targetPose.heading) * 9, targetPose.y + Math.sin(targetPose.heading) * 9);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
        // Update the PurePursuitCommand
        // Get the current position of the robot
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robotPose2 = localizer.getPose();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(robotPose2.x, robotPose2.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(robotPose2.x, robotPose2.y, robotPose2.x + Math.cos(robotPose2.heading) * 9, robotPose2.y + Math.sin(robotPose2.heading) * 9);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetPose.x, targetPose.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(targetPose.x, targetPose.y, targetPose.x + Math.cos(targetPose.heading) * 9, targetPose.y + Math.sin(targetPose.heading) * 9);

        fieldOverlay.strokeCircle(targetPose.x, targetPose.y,2);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        execute();
    }

    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robotPose = localizer.getPose();

        Pose powers = getPower(robotPose);
        drivetrain.setRobotWeightedDrivePower(powers);
    }

    public Pose getPower(Pose robotPose) {
        double currentX = robotPose.x;
        double xPower = -xController.calculate(targetPose.x, currentX);
        if((Math.abs(targetPose.x - currentX) <= 0.08)){
            xPower=0;
        }

        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetPose.heading - currentHeading);
        double hPower = hController.calculate(0, error);
        if((Math.abs(error) <= 0.017453)){
            hPower=0;
        }

        double currentY = robotPose.y;
        double yPower = -yController.calculate(targetPose.y, currentY);
        if((Math.abs(targetPose.y-robotPose.y) <= 0.09)){
            yPower=0;
        }

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        // technically i dont think this is normalized correctly
        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        return new Pose(x_rotated, y_rotated, hPower);
    }
}