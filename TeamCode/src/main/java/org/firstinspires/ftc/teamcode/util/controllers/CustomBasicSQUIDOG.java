package org.firstinspires.ftc.teamcode.util.controllers;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class CustomBasicSQUIDOG extends CustomBasicPIDTweaked {
    public CustomBasicSQUIDOG(PIDCoefficients coefficients) {
        super(coefficients);
    }
//    @Override
//    protected double calculateError(double reference, double state) {
////        return Math.signum(reference-state)*(Math.sqrt(Math.abs(reference-state)));
////        return reference-state;
//    }
    @Override
    public double calculate(double reference, double state) {
        double val = super.calculate(reference, state);
        return Math.signum(val)*(Math.sqrt(Math.abs(val+coefficients.Kd*derivative)));
//        return val+derivative* coefficients.Kd;
    }
}

