package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.localizers.RawOtosLocalizer;
import org.firstinspires.ftc.teamcode.commands.ArmPresetCommand;
import org.firstinspires.ftc.teamcode.commands.CombinedIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendoPresetCommand;
import org.firstinspires.ftc.teamcode.commands.PositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryDrawer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;


@Autonomous
@Config
public class RedAutoMain extends CommandOpMode {

    private RawOtosLocalizer localizer;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private ArmSubsystem arm;
    private DriveSubsystem driveSystem;
    private IntakeSubsystem intake;
    private TelemetryDrawer drawer;
    private double loopTime = 0.0;
    private FtcDashboard dashboard;
    public static double firstPreloadAngle = 71;
    public static double firstPreloadAngle2 = 0;
    public static double specimenAngle = 0;



    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);

        driveSystem = new DriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        localizer=new RawOtosLocalizer(hardwareMap);
        localizer.setPose(0,0,0);


        ParallelCommandGroup scorePreload = new ParallelCommandGroup(
                new PositionCommand(driveSystem, localizer, new Pose(25,0,0)),
                new InstantCommand(() -> {
                    new ArmPresetCommand(arm, (Math.toRadians(firstPreloadAngle))).schedule();
                }),
                new CombinedIntakeCommand(intake, CombinedIntakeCommand.WristPosition.RIGHT, -0.2)
        );

        ParallelCommandGroup goToFirstSample = new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new PositionCommand(driveSystem, localizer, new Pose(12,0,0)),
                    new PositionCommand(driveSystem, localizer, new Pose(36,23, Math.toRadians(90)))
                ),
                new InstantCommand(() -> {
                    new ArmPresetCommand(arm, -10).schedule();
                    new CombinedIntakeCommand(intake, CombinedIntakeCommand.WristPosition.MIDDLE, 0);
                })
        );
//silliness and tomfoolery
        //intakes
        SequentialCommandGroup intakeSample = new SequentialCommandGroup(
                new ExtendoPresetCommand(arm, -200),
                new CombinedIntakeCommand(intake, CombinedIntakeCommand.WristPosition.MIDDLE, -1),
                new WaitCommand(2000),
                new ExtendoPresetCommand(arm, 0)
        );
        //goes to bucket
        ParallelDeadlineGroup goToBucket = new ParallelDeadlineGroup(
                new PositionCommand(driveSystem, localizer, new Pose(10, 40, Math.toRadians(135))),
                new ArmPresetCommand(arm, 1.5),
                new CombinedIntakeCommand(intake, CombinedIntakeCommand.WristPosition.MIDDLE, -0.05)
        );

        TelemetryPacket packet2 = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);
        drawer.drawPose(localizer.getPose(), "#FF0000", 9);
        drawer.drawPose(new Pose(-9,42,Math.toRadians(90)), "#00FF00", 9);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        scorePreload,
                        new WaitCommand(2000),
                        new ExtendoPresetCommand(arm, -1200),
                        new InstantCommand(() -> {
                            new ArmPresetCommand(arm, (Math.toRadians(firstPreloadAngle2))).schedule();
                        }),
                        new WaitCommand(200),
                        new ExtendoPresetCommand(arm,  -0.1),
//                        new CombinedIntakeCommand(intake, CombinedIntakeCommand.WristPosition.RIGHT, ),
                        goToFirstSample,
                        new WaitCommand(2000),
                        intakeSample
                        //new ExtendoPresetCommand(arm,  -0.1),
                       // goToBucket


                )
        );
//        CommandScheduler.getInstance().setDefaultCommand(arm, new ArmPresetCommand(arm, 0.2));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("ms", (loop - loopTime) / 1000000);
//        telemetry.addData("kg", gamepadEx2.getRightY());
        telemetry.addData("angle", Math.toDegrees(arm.armAngle()));
        telemetry.addData("pos", arm.getExtendoPosition());
        telemetry.addData("posex", arm.getExtendoPower());
        telemetry.addData("extendocurrent", arm.getExtendoCurrent());


//        telemetry.addLine(arm.getCurrentCommand().toString());
        TelemetryPacket packet = new TelemetryPacket();
        drawer = new TelemetryDrawer(packet);
        drawer.drawPose(localizer.getPose(), "#FF0000", 9);

        // Drive the robot

        // Send telemetry data to the dashboard
        packet.put("current", arm.armAngle());
        packet.put("target", arm.getArmTarget());
        packet.put("currentangle", arm.armAngle());

        loopTime = loop;
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}