package org.firstinspires.ftc.teamcode.drivegvf;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.drivepp.TwoWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;

import java.util.LinkedList;

@TeleOp(name = "Localization Test Corrected", group = "Test")
@Config
public class GVFHeadingTest extends LinearOpMode {
    private TwoWheelIMULocalizer localizer;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;
    public static CustomBasicPID hController = new CustomBasicPID(new PIDCoefficients(HeadingPIDTest.hP, 0, HeadingPIDTest.hD));
    CubicBezierCurve curve;
    LinkedList<Vector2D> resistingPoints = new LinkedList<>();
    private RBGVFNavigation rbgvfNavigation = new RBGVFNavigation();
    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelIMULocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        curve=new CubicBezierCurve(
                new Vector2D(0, 0),
                new Vector2D(10, 0),
                new Vector2D(10, 10),
                new Vector2D(20, 20));


        waitForStart();

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.getPose();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            // Draw the robot's current pose on the field
            fieldOverlay.setStroke("#FF0000");
            fieldOverlay.strokeCircle(pose.x, pose.y, 9); // 9 is the radius of the robot, adjust as needed
            fieldOverlay.strokeLine(pose.x, pose.y, pose.x + Math.cos(pose.heading) * 9, pose.y + Math.sin(pose.heading) * 9);

            packet.put("x", pose.x);
            packet.put("y", pose.y);
            packet.put("heading", pose.heading);
            packet.put("encoder y", localizer.getParallelEncoderPos());
            packet.put("x velocity", localizer.getVelocity().x);
            packet.put("y velocity", localizer.getVelocity().y);



            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading", pose.heading);
            telemetry.addData("joyy", gamepad1.left_stick_y);
            telemetry.addData("joyx", gamepad1.left_stick_x);
            Vector2D robotPos = new Vector2D(pose.x, pose.y);
            Vector2D calculatedMovementVector = rbgvfNavigation.calculateGuidanceVector(curve, robotPos, resistingPoints);
            double movementDirection = calculatedMovementVector.getHeading();

            double currentHeading = pose.heading;
            double error = AngleUnit.normalizeRadians(movementDirection - currentHeading);
            double hPower = hController.calculate(0, -error);

            drivetrain.setFieldWeightedDrivePower(new Pose(gamepad1.left_stick_x, gamepad1.left_stick_y, hPower), pose.heading);

            fieldOverlay.setStroke("#00FF00");
            fieldOverlay.strokeLine(pose.x, pose.y, pose.x + Math.cos(movementDirection) * 9, pose.y + Math.sin(movementDirection) * 9);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
