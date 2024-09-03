//package org.firstinspires.ftc.teamcode.optimizations;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.drivepp.geometry.Pose;
//import org.firstinspires.ftc.teamcode.drivepp.geometry.PoseVelocity;
//import org.firstinspires.ftc.teamcode.drivepp.geometry.Vector2D;
//import org.firstinspires.ftc.teamcode.legacy.Vector;
//@Config
//public class DrivetrainAlpha {
//    DcMotor FL;
//    DcMotor FR;
//    DcMotor BL;
//    DcMotor BR;
//    public static double lateralMultiplier = 1.0;
//    public static double trackWidth = 1.0;
//    public static double wheelBase = 1.0;
//    public static double VX_WEIGHT = 1.0;
//    public static double VY_WEIGHT = 1.0;
//    public static double OMEGA_WEIGHT = 1.0;
//    double d = (trackWidth + wheelBase) / 2.0;
//
//    public Vector2D centripetalCircleCenterDrawn = null;
//    public Vector2D centripetalCircleRadiusDrawn = null;
//    public Vector2D centripetalVectorDrawn = null;
//    public Vector2D robotDriveDirectionDrawn = null;
//
//    public static double centripetalWeighting = 0.001;
//
//
//    public DrivetrainAlpha(HardwareMap hardwareMap) {
//        this.FL = hardwareMap.get(DcMotor.class, "FL");
//        this.FR = hardwareMap.get(DcMotor.class, "FR");
//        this.BL = hardwareMap.get(DcMotor.class, "BL");
//        this.BR = hardwareMap.get(DcMotor.class, "BR");
//
//        this.FL.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.BL.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//
//    public void setPower(double power) {
//        FL.setPower(power);
//        FR.setPower(power);
//        BL.setPower(power);
//        BR.setPower(power);
//    }
//
//    public void setSplitPower(double leftPower, double rightPower) {
//        FL.setPower(leftPower);
//        BL.setPower(leftPower);
//        FR.setPower(rightPower);
//        BR.setPower(rightPower);
//    }
//
//    public void setRobotWeightedDrivePower(Pose drivePower) {
//        Pose vel = drivePower;
//
//        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.heading);
//        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.heading);
//        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.heading);
//        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.heading);
//    }
//
//    public void setFieldWeightedDrivePower(Pose drivePower, double heading) {
//        Vector fieldFrame = new Vector(drivePower.getX(), drivePower.getY()).rotate(heading);
//        Pose vel = new Pose(fieldFrame.getX(), fieldFrame.getY(), drivePower.heading);
//
//        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.heading) > 1) {
//            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.heading);
//            vel = new Pose(
//                    VX_WEIGHT * drivePower.getX(),
//                    VY_WEIGHT * drivePower.getY(),
//                    OMEGA_WEIGHT * drivePower.heading).scale(1/denom);
//        }
//
//        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.heading);
//        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.heading);
//        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.heading);
//        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.heading);
//    }
//
//    public void setTankPower(double f,double t){
//        FL.setPower(f+t);
//        FR.setPower(f-t);
//        BL.setPower(f+t);
//        BR.setPower(f-t);
//    }
//
//    public void driveWithCorrection(PoseVelocity powers, PoseVelocity currentVelocity, Pose[] poseHistory){
//        Vector2D centripetalPower = calculateCentripetalPower(currentVelocity, poseHistory);
//        Vector2D combinedPower = combinePower(powers.linearVelocity, centripetalPower);
//
//        setRobotWeightedDrivePower(new Pose(combinedPower.x, combinedPower.y, powers.angularVelocity));
//    }
//
//
//    private Vector2D combinePower(Vector2D mainPower, Vector2D correctivePower) {
//        // get max power
//        double maxPower = mainPower.norm();
//
//        // Sum
//        Vector2D combinedPower = mainPower.add(correctivePower);
//
//        // Scale down
//        Vector2D scaledCombinedPower = combinedPower;
//        if (combinedPower.norm() > maxPower) { // scale down if over max. Never scale up
//            scaledCombinedPower = combinedPower.divide(combinedPower.norm()).mult(maxPower);
//        }
//
//        return scaledCombinedPower;
//    }
//
//    private Vector2D calculateCentripetalPower(PoseVelocity currentVelocity, Pose[] poseHistory) {
//        if (poseHistory.length < 3) return new Vector2D(0, 0);
//        Vector2D p3 = poseHistory[2].toVec2D(); // y(t=3)
//        Vector2D p2 = poseHistory[1].toVec2D(); // y(t=2)
//        Vector2D p1 = poseHistory[0].toVec2D(); // y(t=1)
//
//        // Find circumcenter
//        double ax = p1.x;
//        double ay = p1.y;
//        double bx = p2.x;
//        double by = p2.y;
//        double cx = p3.x;
//        double cy = p3.y;
//        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
//        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
//        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
//        Vector2D circumcenter = new Vector2D(ux, uy);
//
//        if (Double.isFinite(circumcenter.x) && Double.isFinite(circumcenter.y)) {
//            centripetalCircleCenterDrawn = circumcenter.mult(1);
//
//            Vector2D circleVector = circumcenter.subt(p3); // represents vector from p3 to the center of the circle
//            centripetalCircleRadiusDrawn = circleVector.mult(1);
//
//            Vector2D centripetalVector = circleVector.divide(circleVector.sqrNorm()); // scales circleVector with mag r to centripetalVector with mag 1/r
//
//            // Calculate power
//            double power = centripetalWeighting * centripetalVector.norm() * currentVelocity.linearVelocity.sqrNorm(); // F=mv^2/r, power = weight*F
//
//            Vector2D centripetalPower = centripetalVector.divide(centripetalVector.norm()).mult(power); // change magnitude to power
//
//            return centripetalPower;
//        } else {
//            centripetalCircleCenterDrawn = null;
//            centripetalCircleRadiusDrawn = null;
//            centripetalVectorDrawn = null;
//            return new Vector2D(0, 0);
//        }
//    }
//
//}
