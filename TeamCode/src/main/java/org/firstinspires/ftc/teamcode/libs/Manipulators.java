package org.firstinspires.ftc.teamcode.libs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Manipulators {
    HardwareMap robot;
    public Servo outtakeServo;
    public Servo droneServo;
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    public static double outtakeServoPos1 = 0.15;
    public static double outtakeServoPos2 = 0;

    public static double droneServoPos1 = 0;

    public static double droneServoPos2 = 0.1;


    public Manipulators(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");

        droneServo = hardwareMap.get(Servo.class, "droneServo");

    }

    // when the method is called, the servo will switch positions and subsequently launch the drone
    public void droneLaunch() {
        droneServo.setPosition(droneServoPos2);
    }

    public void gateToggle(boolean outtakeServoStatus) {
        if (outtakeServoStatus == false){
            outtakeServo.setPosition(outtakeServoPos1); //0.15
        } else if (outtakeServoStatus){
            outtakeServo.setPosition(outtakeServoPos2); //0
        }
    }
}