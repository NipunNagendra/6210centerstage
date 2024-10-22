package org.firstinspires.ftc.teamcode.util;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.robotcore.util.Range;

public class CustomBasicSQUID extends CustomBasicPID {
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

