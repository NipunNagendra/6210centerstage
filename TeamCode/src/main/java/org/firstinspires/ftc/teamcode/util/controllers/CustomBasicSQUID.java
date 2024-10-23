package org.firstinspires.ftc.teamcode.util.controllers;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class CustomBasicSQUID extends CustomBasicPIDTweaked {
    public CustomBasicSQUID(PIDCoefficients coefficients) {
        super(coefficients);
    }

    @Override
    public double calculate(double reference, double state) {
        double val = super.calculate(reference, state);
        return Math.signum(val)*(Math.sqrt(Math.abs(val))-Math.pow((coefficients.Kd*derivative), 2));
//        return val;
    }
}

