package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class WristCommand extends CommandBase {

    // Enum for wrist positions
    public enum WristPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final IntakeSubsystem intakeSubsystem;
    private final WristPosition wristPosition; // Holds the desired wrist position


    public WristCommand(IntakeSubsystem subsystem, WristPosition position) {
        intakeSubsystem = subsystem;
        wristPosition = position;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        switch (wristPosition) {
            case LEFT:
                intakeSubsystem.setWristPosition(0.0);
                break;
            case MIDDLE:
                intakeSubsystem.setWristPosition(0.5);
                break;
            case RIGHT:
                intakeSubsystem.setWristPosition(1.0);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes immediately after setting the wrist position
        return true;
    }
}
