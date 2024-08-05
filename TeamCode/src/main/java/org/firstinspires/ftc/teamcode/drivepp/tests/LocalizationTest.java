package org.firstinspires.ftc.teamcode.drivepp.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.drivepp.TwoWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.drivepp.geometry.Pose;

@TeleOp(name = "Localization Test", group = "Test")
@Config
public class LocalizationTest extends LinearOpMode {
    private TwoWheelIMULocalizer localizer;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelIMULocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();

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


            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading", pose.heading);
            telemetry.addData("joyy", gamepad1.left_stick_y);
            telemetry.addData("joyx", gamepad1.left_stick_x);
            telemetry.update();

            drivetrain.setRobotWeightedDrivePower(new Pose(gamepad1.left_stick_y,
                    gamepad1.left_stick_x, gamepad1.right_stick_x));

            sleep(50); // Update at a reasonable rate
        }
    }
}
