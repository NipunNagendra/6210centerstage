package org.firstinspires.ftc.teamcode.backend.drivegvf;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;

import java.util.List;

public class RBGVFNavigation {

    // Scaling factor based on new dimensions
    private final double SCALE_FACTOR = 0.093; // Example scaling factor

    public Vector2D calculateGuidanceVector(CubicBezierCurve curve, Vector2D currentLocation, List<Vector2D> resistingObjects, Vector2D robotVelocity) {
        long startTime = System.nanoTime();
        double closestT = findClosestPoint(curve, currentLocation);
        Vector2D closestPoint = curve.calculate(closestT);

        Vector2D tangent = curve.derivative(closestT);
        Vector2D normalizedTangent = tangent.normalize();
        double h = 1;
        Vector2D scaledTangent = normalizedTangent.scalarMultiply(h);
        closestPoint = closestPoint.add(scaledTangent);

        Vector2D driveVector = curve.derivative(closestT);
        Vector2D correctionVector = closestPoint.subtract(currentLocation);
        Vector2D centripetalVector = centripetalCorrectiveVector(curve, closestT, robotVelocity, driveVector);
        correctionVector.add(centripetalVector);
        double CORRECTION_DISTANCE = 80 * SCALE_FACTOR; // Scaled down
        Vector2D endPoint = curve.calculate(1);
        double SAVING_THROW_DISTANCE = 100 * SCALE_FACTOR; // Scaled down
        double directPursuitThreshold = 1;
        {
            for (double i = 1; i >= 0; i -= 1 / 100.0) {
                double dist = endPoint.subtract(curve.calculate(i)).getMagSq();
                if (dist > SAVING_THROW_DISTANCE * SAVING_THROW_DISTANCE) { // Squared distance
                    directPursuitThreshold = i;
                    break;
                }
            }
        }
        Vector2D robotToEnd = endPoint.subtract(currentLocation);
        double correctionFactor = Math.min(1, correctionVector.getMagnitude() / CORRECTION_DISTANCE);


        double movementDirection = hlerp(driveVector.getHeading(), correctionVector.getHeading(), correctionFactor);

        if ((closestT == 1 && Math.abs(currentLocation.subtract(closestPoint).getHeading() - driveVector.getHeading()) <= 0.5 * Math.PI) ||
                closestT >= directPursuitThreshold) {
            movementDirection = endPoint.subtract(currentLocation).getHeading();
        }

        Vector2D movementVector = new Vector2D(Math.cos(movementDirection), Math.sin(movementDirection));
        double speed = 1;
        // i'd like to change this logic to distance along the path.
        // maybe use something like euler's method?
        // split the curve into intervals, calculate delta vectors
        // between intervals, store in a lookup table
        if (robotToEnd.getMagnitude() < 200 * SCALE_FACTOR) { // Scaled down
            // speed = lerp(0.2, speed, (robotToEnd.getMagnitude() / 200)*(robotToEnd.getMagnitude() / 200));
            speed = lerp(0.2, speed, robotToEnd.getMagnitude() / (200 * SCALE_FACTOR));
        }

        movementVector = movementVector.scalarMultiply(speed);
        // System.out.println(String.format("Navigation Calculation Took %.3fms", (System.nanoTime() - startTime) / 1e6));
        for (Vector2D resistingObject : resistingObjects) {
            Vector2D toResistingObject = resistingObject.subtract(currentLocation);
            double distance = toResistingObject.getMagnitude();

            if (distance < 100 * SCALE_FACTOR) { // Scaled down
                double normalizedDistance = distance / (100 * SCALE_FACTOR);
                double repulsionStrength = (1 - Math.pow(normalizedDistance, 2)); // Quadratic falloff

                // Perpendicular repulsion
                Vector2D perpendicular = new Vector2D(-tangent.getY(), tangent.getX());
                double perpendicularComponent = toResistingObject.dot(perpendicular);
                Vector2D perpendicularRepulsionVector = perpendicular.scalarMultiply(perpendicularComponent);

                movementVector = movementVector.subtract(perpendicularRepulsionVector.normalize().scalarMultiply(repulsionStrength));

                // Apply parallel repulsion if distance from closest point on curve is below a threshold
                if (currentLocation.subtract(closestPoint).getMagnitude() > 100 * SCALE_FACTOR) { // Scaled down
                    Vector2D parallel = tangent;
                    double parallelComponent = toResistingObject.dot(parallel);
                    Vector2D parallelRepulsionVector = parallel.scalarMultiply(parallelComponent);

                    movementVector = movementVector.subtract(parallelRepulsionVector.normalize().scalarMultiply(repulsionStrength));
                }
            }
        }
//        movementVector = movementVector.add(centripetalVector);
        return movementVector;
    }

    /*
    Hlerp - heading lerp.
    Interpolates between a and b, taking the shortest path across the range
    [-pi, pi] assuming the input range is continuous across said range.

    Say a = -0.9pi, b = 0.9pi. traditional lerp would rotate counterclockwise,
    passing through 0 at t = 0.5. Hlerp will rotate clockwise, passing through
    +/- pi at t = 0.5.

    Hlerp is used to avoid an edge case where the movement direction
    passes through +/- pi as it transitions from to-path correction to
    on-path guidance.
     */
    public double hlerp(double a, double b, double t) {
        double diff = b - a;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return a + t * diff;
    }

    private double lerp(double a, double b, double t) {
        return (1 - t) * a + t * b;
    }

    public double findClosestPoint(CubicBezierCurve curve, Vector2D point) {
        // long startTime = System.nanoTime();
        double minT = -1;
        double minDist = Double.POSITIVE_INFINITY;
        int SAMPLE_DENSITY = 800;
        for (int i = 0; i < SAMPLE_DENSITY + 1; i++) {
            double t = i / (double) SAMPLE_DENSITY;
            double dist = calculateMinimizationFunction(curve, t, point);
            if (dist < minDist) {
                minDist = dist;
                minT = t;
            }
        }
        return minT;
    }

    private double calculateMinimizationFunction(CubicBezierCurve curve, double t, Vector2D point) {
        return curve.calculate(t).subtract(point).getMagSq();
    }

    public Vector2D centripetalCorrectiveVector(CubicBezierCurve curve, double closestT, Vector2D velocity, Vector2D driveVector) {

        double curvature = curve.getCurvature(closestT);

        if (Double.isNaN(curvature)) return new Vector2D(0,0);
        Vector2D centripetalVector = new Vector2D(Range.clip(0.0005 * 10 *
                Math.pow(velocity.dot(driveVector.normalize()), 2)
                * curvature, -1, 1), driveVector.getHeading() + Math.PI / 2 *
                getSign(driveVector.getHeading()));
        return centripetalVector;
    }
    public static double getSign(double get) {
        if (get == 0) return 0;
        if (get > 0) return 1;
        return -1;
    }
}
