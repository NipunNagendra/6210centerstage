package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmPresetCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double targetAngle;

    public ArmPresetCommand(ArmSubsystem armSubsystem, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.moveArmTo(targetAngle);
    }

    @Override
    public boolean isFinished() {

//        return Math.abs(armSubsystem.armAngle() - targetAngle) < 0.05;
        return true;// Ends when arm reaches the target
    }

    @Override
    public void end(boolean interrupted) {
//        armSubsystem.setRawArmPower(0); // Stop arm when command ends
    }
}
