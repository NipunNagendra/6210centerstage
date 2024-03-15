package org.firstinspires.ftc.teamcode.Autos.BlueSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.SensorLibrary;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "StackAutoBlue", group = "Autonomous")
public class StackAutoBlue extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        UNDER_DOOR_OR_TRUSS,
        SCORE_YELLOW,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //copypaste starts here


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -38.15845302224215;
    public static double startPoseY= 65.13672263931143;
    public static double startPoseAngle= Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for right spike position
    public static double spike3X = -44.19633638294297;
    public static double spike3Y = 31.1247700133697;
    public static double spike3Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for left spike position
//    public static double spike3X = -34.26642694740993;
//    public static double spike3Y = 29.54644728121096;
//    public static double spike3Angle = Math.toRadians(180);
    public static double moveBackwards1 = 31;
    public static double moveForward1 = 12;
    public static double turn1 = -90;

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 20;
    public static double dangerPathX = -15;
    public static double trussY = 56.83672263931143;
    public static double trussAngle = Math.toRadians(180);

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 35;
    public static double exitDoor = 15;

    public static double backdropMiddleX = /*51*/ 45.5;
    public static double backdropMiddleY = /*32.58*/ 33;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 5;
    public static double backdropRightStrafe = 5;
    public static double preParkY = 55;
    public static double goingIntoPark = 16;
    public static double temporalMarkerTime = 1.5;

    public static double temporalMarkerTimeAlternate = 4;

    public static double outFromBackdrop = 10;

    public static int outtakeEncoderTicksUp = 1800;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 1.5;
    public static double temporalMarkerTimeDown = 0.5;


    // copypaste ends here


    public static double casenum=1;

    public static BluePipeline.Location positionOfVisionPixel;

    public static String myPosition;
    public static Boolean danger = Boolean.FALSE;

    SensorLibrary sLib;


    State currentState = State.IDLE;


    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        BluePipeline vision =  new BluePipeline(telemetry);


        telemetry.addLine("Init Done");

        drive.setPoseEstimate(startPose);
        //

        //still need to enter values for these
        // these are the basic to spike part
        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .build();

        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
                .back(moveBackwards1)
                .turn(Math.toRadians(turn1))
                .forward(moveForward1)
                .build();

        // post-spike, moving towards prev truss
        TrajectorySequence finishRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .back(5)
//                .turn(Math.toRadians(90))
                .lineToLinearHeading(
                        new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence finishMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .build();
        TrajectorySequence finishLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .back(10)
                .turn(Math.toRadians(180))
                .lineToLinearHeading(
                        new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence underDoorScuffed = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .addTemporalMarker(temporalMarkerTimeAlternate, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .lineToLinearHeading(new Pose2d(dangerPathX, trussY, trussAngle))
                .strafeLeft(
                        betweenTruss,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle), Math.toRadians(90))
                .build();

        // common trajectory for all 3 paths that leads to the backdrop
        TrajectorySequence underTrussToBackdropAll = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .addTemporalMarker(temporalMarkerTime, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                .lineToLinearHeading(
                        new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence strafeToBackdropPosLeft = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeRight(
                        backdropLeftStrafe,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence strafeToBackdropPosRight = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeLeft(
                        backdropRightStrafe,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//        TrajectorySequence parkAll = drive.trajectorySequenceBuilder(posEstimate)
//                .lineToLinearHeading(new Pose2d(backdropMiddleX, preParkY, backdropMiddleAngle))
//                .back(goingIntoPark)
//                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(strafeToBackdropPosLeft.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle + Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .strafeLeft(goingIntoPark)
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(underTrussToBackdropAll.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle + Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .strafeLeft(goingIntoPark)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(strafeToBackdropPosRight.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle + Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .strafeLeft(goingIntoPark)
                .build();

        telemetry.addLine("trajectories built!!!");

        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        BluePipeline detectBlue = new BluePipeline(telemetry);
        camera.setPipeline(detectBlue);

        camera.setMillisecondsPermissionTimeout(5000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {



            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();


        camera.stopStreaming();

        while(!isStopRequested() && opModeIsActive()){

            switch(currentState){
                case IDLE:
//                    manip.moveOuttakeLift(5000);
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    currentState = State.UNDER_DOOR_OR_TRUSS;
                    if (BluePipeline.positionMain == "left") {
                        myPosition="left";
                        telemetry.addLine("going left");
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (BluePipeline.positionMain == "middle") {
                        myPosition="middle";
                        telemetry.addLine("going middle");
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        myPosition="right";
                        telemetry.addLine("going right");
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    telemetry.update();
                    manip.setIntakePower(-0.6);
                    sleep(1000);
                    manip.setIntakePower(0);
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(finishLeft);
                    } else if (myPosition == "middle") {
                        drive.followTrajectorySequence(finishMiddle);
                    } else {
                        drive.followTrajectorySequence(finishRight);
                    }

                    currentState = State.UNDER_DOOR_OR_TRUSS;

                case UNDER_DOOR_OR_TRUSS:
                    // ultrasonic + distance sensor stuff here idk
                    if (/* distance sensor detects robot block is */ danger) {
                        drive.followTrajectorySequence(underDoorScuffed);
                    }

                    else {
                        drive.followTrajectorySequence(underTrussToBackdropAll);
                    }
                    currentState = State.SCORE_YELLOW;

                case SCORE_YELLOW:

                    if (myPosition == "left") {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle);
                        drive.followTrajectorySequence(strafeToBackdropPosLeft);
                    } else if (myPosition == "right") {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, backdropMiddleAngle);
                        drive.followTrajectorySequence(strafeToBackdropPosRight);
                    } else {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle);
                    }
                    sleep(500);
                    manip.gateToggle();
                    sleep(800);
                    currentState = State.PARK;

                case PARK:
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (myPosition == "right") {
                        drive.followTrajectorySequence(parkRight);
                    } else {
                        drive.followTrajectorySequence(parkMiddle);
                    }
                    currentState = State.STOP;

                case STOP:
                    break;
            }
        }

    }

}