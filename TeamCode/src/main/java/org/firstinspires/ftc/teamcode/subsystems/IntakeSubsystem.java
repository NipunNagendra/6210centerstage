package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo activeIntake;
    private final Servo wrist;

    public IntakeSubsystem(final HardwareMap hardwareMap) {
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void setIntakePower(double power) {
        activeIntake.setPower(power);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }


}
