package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.backend.drivepp.Drivetrain;
import org.firstinspires.ftc.teamcode.backend.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmPosCommand;
import org.firstinspires.ftc.teamcode.commands.ArmPresetCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ManualArmControlCommand;
import org.firstinspires.ftc.teamcode.commands.ManualExtendoControlCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryDrawer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

import java.util.function.DoubleSupplier;


@TeleOp
@Config
public class SimpleTeleOp extends CommandOpMode {

    private TwoWheelIMULocalizerLegacy localizer;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private ArmSubsystem arm;
    private DriveSubsystem driveSystem;
    private IntakeSubsystem intake;
    private TelemetryDrawer drawer;
    private double loopTime = 0.0;
    private FtcDashboard dashboard;



    @Override
    public void initialize() {
        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);

        driveSystem = new DriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);


        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().schedule(new RunCommand(() -> {
            localizer.update();
            driveSystem.setHolonomicPower(new Pose(gamepadEx.getLeftY(), gamepadEx.getLeftX(), -gamepadEx.getRightX()));
        }, driveSystem));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
            new WristCommand(intake, WristCommand.WristPosition.RIGHT).schedule();
        })));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
            new WristCommand(intake, WristCommand.WristPosition.LEFT).schedule();
        })));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
            new WristCommand(intake, WristCommand.WristPosition.MIDDLE).schedule();
        })));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new IntakeCommand(intake, 1))
                .whenReleased(new IntakeCommand(intake, 0));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeCommand(intake, -1))
                .whenReleased(new IntakeCommand(intake, 0));

//        CommandScheduler.getInstance().schedule(
//            new ManualArmControlCommand(arm, () -> gamepad2.left_stick_y);
////            new ManualExtendoControlCommand(arm, () -> gamepad2.right_stick_y)));
//            );




//        CommandScheduler.getInstance().schedule(
//                new ManualExtendoControlCommand(arm, () -> (gamepad2.right_trigger-gamepad2.left_trigger)));

        CommandScheduler.getInstance().schedule(
            new ManualArmControlCommand(arm, () -> (gamepad2.right_stick_y), () -> (gamepad2.right_trigger-gamepad2.left_trigger)));


//        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> CommandScheduler.getInstance().schedule(new RunCommand(() -> {
//            new ArmPresetCommand(arm, 0).schedule();
//        })));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        drawer.drawPose(localizer.getPose(), "#FF0000", 9);
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("ms", (loop - loopTime) / 1000000);
        telemetry.addData("kg", gamepadEx2.getRightY());
        telemetry.addData("angle", Math.toDegrees(arm.armAngle()));
        telemetry.addData("pos", arm.getExtendoPosition());
        telemetry.addData("posex", arm.getExtendoPower());

//        telemetry.addLine(arm.getCurrentCommand().toString());
        TelemetryPacket packet = new TelemetryPacket();
        // Drive the robot

        // Send telemetry data to the dashboard
        packet.put("current", arm.armAngle());
        packet.put("target", arm.getArmTarget());


        loopTime = loop;
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}