package org.firstinspires.ftc.teamcode.util.libs;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ultrasonicTest", group="TeleOp")
public class ultrasonicTest extends OpMode {

    MB1242 ultrasonicSensor;

    ModernRoboticsI2cRangeSensor rangeSensor;
    Movement move;

    public void init() {
        ultrasonicSensor = hardwareMap.get(MB1242.class, "ultrasonicSensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        ultrasonicSensor.ping();
        move = new Movement(hardwareMap);
        telemetry.addData("distance: ", ultrasonicSensor.getDistance(DistanceUnit.CM));
        telemetry.update();



    }

    @Override
    public void loop() {
        if (move.isPressed("x", gamepad1.x)){
            ultrasonicSensor.ping();
        }
        if (move.isPressed("y", gamepad1.y)){
            telemetry.addData("distance: ", ultrasonicSensor.getDistance(DistanceUnit.CM));
        }
        telemetry.addData("range: ultra", rangeSensor.cmUltrasonic());
        telemetry.addData("range: light", rangeSensor.cmOptical());
        telemetry.update();

    }


}
