package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
@Config
public class WristCommand extends CommandBase {

    // Enum for wrist positions
    public enum WristPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final IntakeSubsystem intakeSubsystem;
    private final WristPosition wristPosition; // Holds the desired wrist position
    public static double leftPos = 0.0;
    public static double midPos = 0.5;
    public static double rightPos = 1.0;


    public WristCommand(IntakeSubsystem subsystem, WristPosition position) {
        intakeSubsystem = subsystem;
        wristPosition = position;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        switch (wristPosition) {
            case LEFT:
                intakeSubsystem.setWristPosition(leftPos);
                break;
            case MIDDLE:
                intakeSubsystem.setWristPosition(midPos);
                break; // break nipun nagendra's brain
            case RIGHT:
                intakeSubsystem.setWristPosition(rightPos);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes immediately after setting the wrist position
        return true;
    }
}
