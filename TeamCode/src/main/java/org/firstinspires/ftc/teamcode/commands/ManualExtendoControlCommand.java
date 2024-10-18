package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ManualExtendoControlCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double joystickInput; // This is the joystick input (between -1.0 and 1.0)

    private static final double MAX_POSITION = 2987;  // The max position for the slide
    private static final double MIN_POSITION = 0;     // The min position for the slide
    private static final double SPEED_MULTIPLIER = 0.5; // Adjust this for how fast you want the slide to move

    public ManualExtendoControlCommand(ArmSubsystem armSubsystem, double joystickInput) {
        this.armSubsystem = armSubsystem;
        this.joystickInput = joystickInput;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double currentPosition = armSubsystem.getExtendoPosition();

        // Calculate the desired power based on the joystick input and scaling it
        double power = joystickInput * SPEED_MULTIPLIER;

        // Prevent overextension
        if ((currentPosition >= MAX_POSITION && power > 0) || (currentPosition <= MIN_POSITION && power < 0)) {
            armSubsystem.setRawExtendoPower(0);  // Stop the motor if it's out of bounds
        } else {
            armSubsystem.setRawExtendoPower(power);  // Otherwise, apply the power
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run this command until it's interrupted or cancelled
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setRawExtendoPower(0); // Stop the motor when the command ends
    }
}
