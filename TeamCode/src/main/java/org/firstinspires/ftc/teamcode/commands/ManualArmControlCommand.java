package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ManualArmControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier stickInput;
    private final DoubleSupplier stickInputx;
    private boolean power_on = false;


    public ManualArmControlCommand(ArmSubsystem armSubsystem, DoubleSupplier stickInput, DoubleSupplier stickInputx) {
        this.armSubsystem = armSubsystem;
        this.stickInput = stickInput;
        this.stickInputx = stickInputx;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.manualControl(stickInput.getAsDouble());
        double current = armSubsystem.getExtendoCurrent();
//
//        if (power_on && current > 1){
//            power_on=false;
//            armSubsystem.setRawExtendoPower(0);
//        }
//        else if (!power_on && current<0.2) {
//            power_on=true;
//        }
        armSubsystem.setRawExtendoPower(Range.clip((stickInputx.getAsDouble() - (0.07+ current)),0,1));


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
