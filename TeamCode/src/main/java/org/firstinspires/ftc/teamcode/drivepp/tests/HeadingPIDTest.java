package org.firstinspires.ftc.teamcode.drivepp.tests;

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
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.CustomBasicPID;

@Config
@Autonomous
public class HeadingPIDTest extends OpMode {

    private Drivetrain drivetrain;
    private TwoWheelIMULocalizer localizer;
    private ElapsedTime runtime;
    private FtcDashboard dashboard;


    public static double hP = 0.4;
    public static double hI = 0;
    public static double hD = 0.3;
    public static double targetHeading = 0.0;
    public static CustomBasicPID headingController;

    @Override
    public void init() {
        headingController = new CustomBasicPID(new PIDCoefficients(hP, hI, hD));
        localizer = new TwoWheelIMULocalizer(hardwareMap);
        localizer.setPose(0,0,0);
        dashboard = FtcDashboard.getInstance();
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        // Update the PurePursuitCommand
        // Get the current position of the robot
        TelemetryPacket packet = new TelemetryPacket();

        Pose robotPose = localizer.getPose();
        headingController.setCoefficients(new PIDCoefficients(hP, hI,hD));
        // Update the PID controller with the current position and target
        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
        double headingPower = headingController.calculate(0, error);
//        if(!(Math.abs(error) <= 0.017453)){
//        }
        drivetrain.setRobotWeightedDrivePower(new Pose(0, 0, headingPower));

        // Drive the robot

        // Send telemetry data to the dashboard
        packet.put("Current Heading", currentHeading);
        packet.put("Target Heading", targetHeading);
        packet.put("PID Output", headingPower);
        packet.put("PID P", hP);
        packet.put("PID I", hI);
        packet.put("PID D", hD);
        packet.put("error", targetHeading-robotPose.heading);

        if(gamepad1.dpad_up){
            targetHeading = 0;
        }
        else if (gamepad1.dpad_down){
            targetHeading=3.14159;
        }

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
