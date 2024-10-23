package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.backend.localizers.RawOtosLocalizer;
import org.firstinspires.ftc.teamcode.backend.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;

@TeleOp(name = "Localization Test OTOS", group = "Test")
@Config
public class LocalizationTestOTOS extends LinearOpMode {
    private RawOtosLocalizer localizer;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;
    ElapsedTime loopTime = new ElapsedTime();
    double multiplier = 1;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new RawOtosLocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
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
//            packet.put("encoder y", localizer.getParallelEncoderPos());
//            packet.put("x velocity", localizer.getVelocity().x);
//            packet.put("y velocity", localizer.getVelocity().y);


            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading", pose.heading);
            telemetry.addData("joyy", gamepad1.left_stick_y);
            telemetry.addData("joyx", gamepad1.left_stick_x);
            telemetry.addData("looptime", loopTime.milliseconds());
            telemetry.update();

            if(gamepad1.right_bumper){
                multiplier=0.5;
            }
            else{
                multiplier=1;
            }
            Vector2D drive = new Vector2D(gamepad1.left_stick_y*multiplier, gamepad1.left_stick_x*multiplier);
            drive.setX(0.5*Math.tan(1.12*drive.x));
            drive.setY(0.5*Math.tan(1.12*drive.y));
//            drive.setX(0.5*(Math.pow(drive.x,3)+drive.x));
//            drive.setY(0.5*(Math.pow(drive.y,3)+drive.y));
            drivetrain.setRobotWeightedDrivePower(new Pose(drive.x,
                    drive.y, gamepad1.right_stick_x*multiplier));

            loopTime.reset();

        }
    }
}
