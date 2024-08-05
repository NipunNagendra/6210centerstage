package org.firstinspires.ftc.teamcode.drivepp.tests;

import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hD;
import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hI;
import static org.firstinspires.ftc.teamcode.drivepp.tests.HeadingPIDTest.hP;
import static org.firstinspires.ftc.teamcode.drivepp.tests.XPIDTest.xD;
import static org.firstinspires.ftc.teamcode.drivepp.tests.XPIDTest.xI;
import static org.firstinspires.ftc.teamcode.drivepp.tests.XPIDTest.xP;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.drivepp.TwoWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.drivepp.geometry.Pose;
import org.firstinspires.ftc.teamcode.drivepp.util.CustomBasicPID;

@Config
@Autonomous
public class YPIDTest extends OpMode {

    private Drivetrain drivetrain;
    private TwoWheelIMULocalizer localizer;
    private ElapsedTime runtime;
    private FtcDashboard dashboard;


    public static double yP = 0.06;
    public static double yI = 0;
    public static double yD = 0.06;
    public static double targetY = 0.0;
    public static double targetX = 0.0;
    public static double targetHeading = 0.0;



    public static CustomBasicPID xController;

    public static CustomBasicPID headingController;
    public static CustomBasicPID yController;


    @Override
    public void init() {
        xController = new CustomBasicPID(new PIDCoefficients(xP, xI, xD));
        localizer = new TwoWheelIMULocalizer(hardwareMap);
        localizer.setPose(0,0,0);
        dashboard = FtcDashboard.getInstance();
        drivetrain = new Drivetrain(hardwareMap);
        headingController = new CustomBasicPID(new PIDCoefficients(hP, hI, hD));
        yController = new CustomBasicPID(new PIDCoefficients(yP, yI, yD));

    }

    @Override
    public void loop() {
        // Update the PurePursuitCommand
        // Get the current position of the robot
        TelemetryPacket packet = new TelemetryPacket();
        Pose robotPose = localizer.getPose();

        double currentX = robotPose.x;
        double xPower = xController.calculate(targetX, currentX);
        if((Math.abs(0 -currentX) <= 0.08)){
            xPower=0;
        }

        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
        double headingPower = headingController.calculate(0, error);
        if((Math.abs(error) <= 0.017453)){
            headingPower=0;
        }

        yController.setCoefficients(new PIDCoefficients(yP, yI,yD));
        double currentY = robotPose.y;
        double yPower = yController.calculate(targetY, currentY);
        if((Math.abs(targetY-robotPose.y) <= 0.09)){
            yPower=0;
        }
        // Drive the robot
        drivetrain.setRobotWeightedDrivePower(new Pose(xPower, yPower, headingPower));

        // Send telemetry data to the dashboard
        packet.put("Current Y", currentY);
        packet.put("Target Y", targetY);
        packet.put("PID Output", yPower);
        packet.put("PID P", yP);
        packet.put("PID I", yI);
        packet.put("PID D", yD);

        Canvas fieldOverlay = packet.fieldOverlay();

        // Draw the robot's current pose on the field
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
