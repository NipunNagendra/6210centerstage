package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualArmControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier stickInput;

    public ManualArmControlCommand(ArmSubsystem armSubsystem, DoubleSupplier stickInput) {
        this.armSubsystem = armSubsystem;
        this.stickInput = stickInput;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.manualControl(stickInput.getAsDouble());

    }


    @Override
    public void end(boolean interrupted) {
        armSubsystem.setRawArmPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
