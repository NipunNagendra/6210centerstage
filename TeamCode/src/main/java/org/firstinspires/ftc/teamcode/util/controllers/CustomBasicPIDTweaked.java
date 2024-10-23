package org.firstinspires.ftc.teamcode.util.controllers;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.robotcore.util.Range;

public class CustomBasicPIDTweaked implements FeedbackController {

    PIDCoefficients coefficients;

    protected boolean hasRun = false;

    protected Timer timer = new Timer();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    // For low-pass filter
    protected double filteredReference = 0;
    protected double alpha = 0.1; // Smoothing factor, adjust as needed

    public CustomBasicPIDTweaked(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public void setCoefficients(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    /**
     * calculate PID output
     * @param reference the target position
     * @param state current system state
     * @return PID output
     */
    @Override
    public double calculate(double reference, double state) {
        double dt = getDT();

        // Apply low-pass filter to the reference
        filteredReference = alpha * reference + (1 - alpha) * filteredReference;

        double error = calculateError(filteredReference, state);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;

        return error * coefficients.Kp
                + integralSum * coefficients.Ki;
//                + derivative * coefficients.Kd;
    }

    /**
     * get the time constant
     * @return time constant
     */
    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.currentTime();
        timer.reset();
        return dt;
    }

    protected double calculateError(double reference, double state) {
//        return Math.signum(reference-state)*(Math.sqrt(Math.abs(reference-state)));
          return reference-state;
    }

    protected void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
        integralSum = Range.clip(integralSum, -1.0 / coefficients.Ki, 1.0 / coefficients.Ki);
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }
}
