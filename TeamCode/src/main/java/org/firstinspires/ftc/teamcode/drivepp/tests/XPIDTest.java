package org.firstinspires.ftc.teamcode.drivepp.tests;

import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hD;
import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hI;
import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
@Config
@Autonomous
public class XPIDTest extends OpMode {

    private Drivetrain drivetrain;
    private TwoWheelIMULocalizerLegacy localizer;
    private ElapsedTime runtime;
    private FtcDashboard dashboard;


    public static double xP = 0.04;
    public static double xI = 0;
    public static double xD = 0.03;
    public static double targetX = 0.0;
    public static CustomBasicPID xController;

    public static CustomBasicPID headingController;

    @Override
    public void init() {
        xController = new CustomBasicPID(new PIDCoefficients(xP, xI, xD));
        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        localizer.setPose(0,0,0);
        dashboard = FtcDashboard.getInstance();
        drivetrain = new Drivetrain(hardwareMap);
        headingController = new CustomBasicPID(new PIDCoefficients(hP, hI, hD));

    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        xController.setCoefficients(new PIDCoefficients(xP, xI,xD));
        Pose robotPose = localizer.getPose();

        double currentX = robotPose.x;
        double xPower = xController.calculate(targetX, currentX);


        // Update the PID controller with the current position and target
        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(0 - currentHeading);
        double headingPower = headingController.calculate(0, error);

        drivetrain.setRobotWeightedDrivePower(new Pose(xPower, 0, headingPower));

        // Send telemetry data to the dashboard
        packet.put("Current X", currentX);
        packet.put("Target X", targetX);
        packet.put("PID Output", xPower);
        packet.put("PID P", xP);
        packet.put("PID I", xI);
        packet.put("PID D", xD);

        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(robotPose.x, robotPose.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(robotPose.x, robotPose.y, robotPose.x + Math.cos(robotPose.heading) * 9, robotPose.y + Math.sin(robotPose.heading) * 9);
        telemetry.update();
        localizer.update();
        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void stop() {
        // Stop the robot
        drivetrain.setRobotWeightedDrivePower(new Pose(0, 0, 0));
    }

}
