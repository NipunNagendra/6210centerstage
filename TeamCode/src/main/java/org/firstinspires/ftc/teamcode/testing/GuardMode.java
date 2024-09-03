package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.driverr.SampleMecanumDrive;
@Config
public class GuardMode extends LinearOpMode {
   public static double xyParameter = .2;
    public static double headingParameter = .2;
    double lockThreshold = 0;
    public static double lockX = 0;
    public static double locky = 0;
    public static double maxVectorMagnitude = 5;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            Vector2d driveVector = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            driveVector = driveVector.rotated(drive.getExternalHeading());
            drive.setPoseEstimate(new Pose2d(0,0,0));
            Pose2d lockLocation = new Pose2d(driveVector.getX()*maxVectorMagnitude, driveVector.getY()*maxVectorMagnitude, -gamepad1.right_stick_x*maxVectorMagnitude);

            lockTo(lockLocation, drive);
            drive.update();

        }
    }

    public void lockTo(Pose2d targetPos, SampleMecanumDrive drive) {
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(new Pose2d(xy.times(xyParameter), heading* headingParameter));
    }}

