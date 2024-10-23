package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.drivegvf.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.backend.drivegvf.RBGVFNavigation;
import org.firstinspires.ftc.teamcode.backend.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.backend.drivepp.tests.HeadingPIDTest;
import org.firstinspires.ftc.teamcode.backend.drivepp.tests.XPIDTest;
import org.firstinspires.ftc.teamcode.backend.drivepp.tests.YPIDTest;
import org.firstinspires.ftc.teamcode.backend.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.controllers.CustomBasicPID;
import org.firstinspires.ftc.teamcode.util.TelemetryDrawer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;

import java.util.LinkedList;

@TeleOp(name = "Localization Test Updated", group = "Test")
@Config
public class LocalizationTestUpdated extends LinearOpMode {
    private TwoWheelLocalizer localizer;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;
    public static CustomBasicPID hController = new CustomBasicPID(new PIDCoefficients(HeadingPIDTest.hP, 0, HeadingPIDTest.hD));
    public static CustomBasicPID xController = new CustomBasicPID(new PIDCoefficients(XPIDTest.xP, 0, XPIDTest.xD));
    public static CustomBasicPID yController = new CustomBasicPID(new PIDCoefficients(YPIDTest.yP, 0, YPIDTest.yD));
    CubicBezierCurve curve;
    LinkedList<Vector2D> resistingPoints = new LinkedList<>();
    private RBGVFNavigation rbgvfNavigation = new RBGVFNavigation();
    private TelemetryDrawer drawer; // Declare the TelemetryDrawer

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelLocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        // Initialize the TelemetryDrawer
        TelemetryPacket packet = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.getPoseEstimate();
            Pose predictedPose = localizer.getExpectedPose();

            // Clear previous drawings
            packet = new TelemetryPacket();
            drawer = new TelemetryDrawer(packet);

            // Draw the robot's current pose on the field
            drawer.drawPose(pose, "#FF0000", 9);
            drawer.drawPose(predictedPose, "#00FF00", 9);

            packet.put("x", pose.x);
            packet.put("y", pose.y);
            packet.put("heading", pose.heading);
            packet.put("x velocity", localizer.getRelativePoseVelocity().x);
            packet.put("y velocity", localizer.getRelativePoseVelocity().y);

            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading", pose.heading);
            telemetry.addData("joyy", gamepad1.left_stick_y);
            telemetry.addData("joyx", gamepad1.left_stick_x);

            // acc code
            double x = 0;

//            drivetrain.setFieldWeightedDrivePower(new Pose(x, , hPower), 0);


            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
