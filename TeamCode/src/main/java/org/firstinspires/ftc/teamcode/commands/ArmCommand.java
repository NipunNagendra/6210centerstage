package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier armPower;
    private final double k_g = 0;


    public ArmCommand(ArmSubsystem subsystem, DoubleSupplier armPower) {
        this.armSubsystem = subsystem;
        this.armPower = armPower;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        // Sets power to the intake motor and position to the wrist servo based on the control inputs
        armSubsystem.setRawArmPower(armPower.getAsDouble()+0.2);
    }

    @Override
    public void end(boolean interrupted) {
        // Stops the motors when the command ends
        armSubsystem.setRawArmPower(0);
    }

    @Override
    public boolean isFinished() {
        // This command never ends unless interrupted
        return false;
    }
}
