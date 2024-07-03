package org.firstinspires.ftc.teamcode.legacy;

public class pathCalculuation {
    //TODO: Check this bruh
    public static void calculateCurvature(Waypoint[] points) {
        double k1;
        double k2;
        double a;
        double b;
        double r;
        for (int i = 0; i < points.length; i++) {
            Waypoint current = points[i];
            if (current.endpoint) {
                current.setCurvature(0);
                System.out.println("end");
            } else {
                Waypoint previous = points[i-1];
                Waypoint next = points[i+1];
                k1 = 0.5 * ((square(previous.x) + square(previous.y)
                        - square(current.x) - square(current.y))/((previous.x + 0.001) - current.x));
                k2 = (previous.y - current.y)/((previous.x + 0.001) - current.x);
                b =  0.5 *((square(current.x) - 2 * current.x * k1 + square(current.y) - square(next.x) + 2 * next.x * k1 - square(next.y))
                        /(next.x*k2 - next.y + current.y - current.x*k2));
                a = (k1-k2*b);
                r = Math.sqrt(square((previous.x - a)) - square((previous.y-b)));
                if (Double.isNaN(1/r)){
                    current.setCurvature(0);
                }
                else{
                    current.setCurvature(1/r);
                }
            }
        }
    }

    public static void calculateVelocity(Waypoint[] points, double maxVelocity, double maxAcceleration, double k){
        for (int i = points.length-1; i >= 0; i--){
            Waypoint current = points[i];
            if (i == points.length-1){
                current.velocity = 0;
            }
            else{
                Waypoint next = points[i+1];
                double distance = calculateDistance(next.x - current.x, next.y - current.y);
                double velocity = Math.min(Math.min(maxVelocity, (k/(current.curvature+0.001))),
                        Math.sqrt(square(next.velocity) + 2 * maxAcceleration * distance));
                current.velocity = velocity;
            }
        }
    }
    public static double square(double x){
        return x*x;
    }
    public static double calculateDistance(double x, double y){
        return Math.sqrt(square(x) + square(y));
    }
}