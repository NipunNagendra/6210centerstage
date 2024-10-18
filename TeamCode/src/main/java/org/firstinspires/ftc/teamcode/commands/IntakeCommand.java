package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {

        private final IntakeSubsystem intakeSubsystem;
        private final double intakePowerSupplier;


        /**
        * Creates a new IntakeCommand.
        *
        * @param subsystem The intake subsystem used by this command.
        * @param intakePower The control input for moving the intake motor.

        */
        public IntakeCommand(IntakeSubsystem subsystem, double intakePower) {
            intakeSubsystem = subsystem;
            intakePowerSupplier = intakePower;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            // Sets power to the intake motor and position to the wrist servo based on the control inputs
            intakeSubsystem.setIntakePower(intakePowerSupplier);
        }

        @Override
        public void end(boolean interrupted) {
            // Stops the motors when the command ends
            intakeSubsystem.setIntakePower(0);
        }

        @Override
        public boolean isFinished() {
            // This command never ends unless interrupted
            return false;
        }
}
