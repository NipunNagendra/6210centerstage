package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualExtendoControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier stickInput;

    public ManualExtendoControlCommand(ArmSubsystem armSubsystem, DoubleSupplier stickInput) {
        this.armSubsystem = armSubsystem;
        this.stickInput = stickInput;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setRawExtendoPower(stickInput.getAsDouble());
    }


    @Override
    public void end(boolean interrupted) {
        armSubsystem.setRawExtendoPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
