package org.firstinspires.ftc.teamcode.util.libs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SensorLibrary {
    MB1242 Mb;
    public MB1242 ultrasonicSensor;

    public Rev2mDistanceSensor distanceSensor;

    public static double dsInvisibleWallThreshold = 0;

    public static double ultrasonicInvisibleWallThreshold = 0;

    public static double dsRobotBlockThreshold = 0;

    public static double ultrasonicRobotBlockThreshold = 0;



    public SensorLibrary(HardwareMap hardwareMap) {
        ultrasonicSensor = hardwareMap.get(MB1242.class, "ultrasonicSensor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    public double getUltrasonicDistance() {
        return Mb.getDistance(DistanceUnit.CM);
    }

    public boolean robotBlockDetect() {
        if (distanceSensor.getDistance(DistanceUnit.CM) <= dsRobotBlockThreshold &&
                ultrasonicSensor.getDistance(DistanceUnit.CM) <= ultrasonicRobotBlockThreshold){
            return true;
        }
        return false;
    }

    public double getDSDistance(){return distanceSensor.getDistance(DistanceUnit.CM);}

    public boolean invisibleWallDetect()
    {
        if (distanceSensor.getDistance(DistanceUnit.CM) < dsInvisibleWallThreshold && ultrasonicSensor.getDistance(DistanceUnit.CM) < ultrasonicInvisibleWallThreshold){
            return true;
        }
        return false;
    }
}

