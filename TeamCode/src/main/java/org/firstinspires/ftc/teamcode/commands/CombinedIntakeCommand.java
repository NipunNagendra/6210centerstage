package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
public class CombinedIntakeCommand extends CommandBase {

    // Enum for wrist positions
    public enum WristPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final IntakeSubsystem intakeSubsystem;
    private final WristPosition wristPosition; // Holds the desired wrist position
    public static double leftPos = 0.0;
    public static double midPos = 0;
    public static double rightPos = 0.26;
    private static double intakePowerSupplier;



    public CombinedIntakeCommand(IntakeSubsystem subsystem, WristPosition position, double intakePower) {
        intakeSubsystem = subsystem;
        wristPosition = position;
        intakePowerSupplier = intakePower;
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
        intakeSubsystem.setIntakePower(intakePowerSupplier);
    }

    @Override
    public boolean isFinished() {
        // Command finishes immediately after setting the wrist position
        return true;
    }
}
