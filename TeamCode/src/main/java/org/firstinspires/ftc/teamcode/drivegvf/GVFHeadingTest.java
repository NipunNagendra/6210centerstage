package org.firstinspires.ftc.teamcode.drivegvf;

import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.X_GAIN;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.drivepp.tests.XPIDTest;
import org.firstinspires.ftc.teamcode.drivepp.tests.YPIDTest;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;
import org.firstinspires.ftc.teamcode.util.TelemetryDrawer; // Import the TelemetryDrawer class

import java.util.LinkedList;

@TeleOp(name = "GVF Test", group = "Test")
@Config
public class GVFHeadingTest extends LinearOpMode {
    private TwoWheelIMULocalizerLegacy localizer;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;
    public static CustomBasicPID hController;
    public static CustomBasicPID xController = new CustomBasicPID(new PIDCoefficients(XPIDTest.xP, 0, XPIDTest.xD));
    public static CustomBasicPID yController = new CustomBasicPID(new PIDCoefficients(YPIDTest.yP, 0, YPIDTest.yD));
    CubicBezierCurve curve;
    LinkedList<Vector2D> resistingPoints = new LinkedList<>();
    private RBGVFNavigation rbgvfNavigation = new RBGVFNavigation();
    private TelemetryDrawer drawer; // Declare the TelemetryDrawer
    public static double startX=0;
    public static double startY=0;
    public static double startH=0;
    public static double power=1;


    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        localizer.setPose(startX, startY, startH);
        dashboard = FtcDashboard.getInstance();
        curve = new CubicBezierCurve(
                new Vector2D(0, 0),
                new Vector2D(50, 0),
                new Vector2D(30, 30),
                new Vector2D(60, 60));

        // Initialize the TelemetryDrawer
        TelemetryPacket packet = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);
        drawer.drawBezierCurve(curve, "#00FF00");
        dashboard.sendTelemetryPacket(packet);
        hController = new CustomBasicPID(new PIDCoefficients(0.4, 0, 0.3));


        waitForStart();

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.getPose();

            // Clear previous drawings
            packet = new TelemetryPacket();
            drawer = new TelemetryDrawer(packet);

            // Draw the robot's current pose on the field
            drawer.drawPose(pose, "#FF0000", 9);

            packet.put("x", pose.x);
            packet.put("y", pose.y);
            packet.put("heading", pose.heading);
            packet.put("x velocity", localizer.getVelocity().x);
            packet.put("y velocity", localizer.getVelocity().y);

            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading", pose.heading);
            telemetry.addData("joyy", gamepad1.left_stick_y);
            telemetry.addData("joyx", gamepad1.left_stick_x);

            // acc code
            Vector2D robotPos = new Vector2D(pose.x, pose.y);
            Vector2D calculatedMovementVector = rbgvfNavigation.calculateGuidanceVector(curve, robotPos, resistingPoints, localizer.getVelocity());
            double movementDirection = (calculatedMovementVector.getHeading());

            double currentHeading = pose.heading;
            double error = AngleUnit.normalizeRadians(movementDirection - currentHeading);
            double hPower = hController.calculate(0, error);
            packet.put("hpower", hPower);

            double xPower = calculatedMovementVector.x * power;
            double yPower = calculatedMovementVector.y * power;

            double x_rotated = xPower * Math.cos(-pose.heading) - yPower * Math.sin(-pose.heading);
            double y_rotated = xPower * Math.sin(-pose.heading) + yPower * Math.cos(-pose.heading);

            hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
            x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
            y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

//            drivetrain.setFieldWeightedDrivePower(new Pose(0, 0, hPower), 0);
            drivetrain.setRobotWeightedDrivePower(new Pose(x_rotated, y_rotated, hPower));

            drawer.drawBezierCurve(curve, "#00FF00");
            drawer.drawPose(new Pose(pose.x + Math.cos(movementDirection) * 9, pose.y + Math.sin(movementDirection) * 9, movementDirection), "#00FF00", 9);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
