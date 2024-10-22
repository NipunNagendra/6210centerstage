//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.testing.GuardMode.headingParameter;
//import static org.firstinspires.ftc.teamcode.testing.GuardMode.maxVectorMagnitude;
//import static org.firstinspires.ftc.teamcode.testing.GuardMode.xyParameter;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.util.Angle;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.backend.drivepp.Drivetrain;
//import org.firstinspires.ftc.teamcode.backend.driverr.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.backend.localizers.TwoWheelIMULocalizerLegacy;
//import org.firstinspires.ftc.teamcode.util.geometry.Pose;
//import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;
//
//@Config
//@TeleOp
//public class GuardMode extends LinearOpMode {
//    private TwoWheelIMULocalizerLegacy localizer;
//    private FtcDashboard dashboard;
//    private Drivetrain drivetrain;
//    ElapsedTime loopTime = new ElapsedTime();
//    double multiplier = 1;
//
//    @Override
//    public void runOpMode() {
//        drivetrain = new Drivetrain(hardwareMap);
//        localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
//        dashboard = FtcDashboard.getInstance();
//
//        waitForStart();
//
//        public void runOpMode() throws InterruptedException {
//            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//            waitForStart();
//            while (opModeIsActive()) {
//                Vector2D driveVector = new Vector2D(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
//                driveVector = driveVector.rotate(drive.getExternalHeading());
//                Pose lockLocation = new Pose(driveVector.getX()*maxVectorMagnitude, driveVector.getY()*maxVectorMagnitude, -gamepad1.right_stick_x*maxVectorMagnitude);
//
//                lo(lockLocation, drivetrain);
//                drive.update();
//
//            }
//        }
//
//        public void lockTo(Pose targetPos, Drivetrain drive) {
//            Pose currPos = drive.getPoseEstimate();
//            Pose difference = targetPos.minus(currPos);
//            Vector2D xy = difference.vec().rotated(-currPos.getHeading());
//            double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());
//
//            drivetrain.setFieldWeightedDrivePower(new Pose2d(xy.mult(xyParameter), heading* headingParameter), 0);
//        }}
//
