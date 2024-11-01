package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ExtendoPresetCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double targetTicks;

    public ExtendoPresetCommand(ArmSubsystem armSubsystem, double targetTicks) {
        this.armSubsystem = armSubsystem;
        this.targetTicks = targetTicks;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.moveExtendoTo(targetTicks);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getExtendoPosition() - targetTicks) < 10;
//        return true;// Ends when arm reaches the target
    }

    @Override
    public void end(boolean interrupted) {
//        armSubsystem.setRawExtendoPower(0); // Stop arm when command ends
    }
}
