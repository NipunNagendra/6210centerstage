package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class drivemotortest extends OpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    @Override
    public void init(){
        //Whats in the quotes should be what you named in the configuration
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            FL.setPower(1);
            telemetry.addLine("FL is moving forward");
        }
        else if(gamepad1.dpad_right){
            FR.setPower(1);
            telemetry.addLine("FR is moving forward");
        }
        else if(gamepad1.dpad_down){
            BR.setPower(1);
            telemetry.addLine("BR is moving forward");
        }
        else if(gamepad1.dpad_left){
            BL.setPower(1);
            telemetry.addLine("BL is moving forward");
        }
        else if(gamepad1.triangle){
            FL.setPower(-1);
            telemetry.addLine("FL is moving backward");
        }
        else if(gamepad1.circle){
            FR.setPower(-1);
            telemetry.addLine("FR is moving backward");
        }
        else if(gamepad1.cross){
            BR.setPower(-1);
            telemetry.addLine("BR is moving backward");
        }
        else if(gamepad1.square){
            BL.setPower(-1);
            telemetry.addLine("BL is moving backward");
        }
        else{
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            telemetry.addLine("All motors are stopped");
        }

    }

}
