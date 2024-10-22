package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualArmControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier stickInput;
    private final DoubleSupplier stickInputx;

    public ManualArmControlCommand(ArmSubsystem armSubsystem, DoubleSupplier stickInput, DoubleSupplier stickInputx) {
        this.armSubsystem = armSubsystem;
        this.stickInput = stickInput;
        this.stickInputx = stickInputx;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.manualControl(stickInput.getAsDouble());
        armSubsystem.setRawExtendoPower(stickInputx.getAsDouble());
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
