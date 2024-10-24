package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

public class ArmPosCommand extends CommandBase {

    private static final double ARM_SPEED = 0.4;
    private final ArmSubsystem armSubsystem;
    private final double armPos;
    public static double ARM_F = 0.1;
    public static double TICK_PER_RAD = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28) / 2*Math.PI / .3333;
    // CHANGE THIS: The "/ 3.2" at the end of the previous line is the gear ratio, since we were using a 32 tooth sprocket on the arm and a 10 tooth sprocket on the motor
    // Make sure to initialize the robot with the arm resting inside the robot

    public static double ARM_TARGET = 0.0;
    public static double ARM_OFF = -2.01;
    private double armAngle() {
        return (armSubsystem.getArmPosition() -  armStart)/TICK_PER_RAD - ARM_OFF;
    }
    private double armStart = 0.0;


    static class ArmPIDF implements FeedForwardConstant {
        @Override
        public double getConstant(double input) {
            return Math.sin(input) * ARM_F;
        }
    }
    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(1, 0.02, 0.02, ARM_F);


    private PIDFController armPID = new PIDFController(PID);


    public ArmPosCommand(ArmSubsystem subsystem, double armPos) {
        this.armSubsystem = subsystem;

        this.armPos = armPos;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armPID.updateFeedForwardInput(armAngle());
        armPID.setTargetPosition(ARM_TARGET);
        armPID.updatePosition(armAngle());
        armSubsystem.setRawArmPower(armPID.runPIDF());
        ARM_TARGET -= armPos * ARM_SPEED;
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
