package org.firstinspires.ftc.teamcode.util;

public interface FeedForwardConstant {

    /**
     * This returns the coefficient for the feedforward factor.
     *
     * @param input this is inputted into the feedforward equation, if applicable. If there's no
     *              equation, then any input can be used.
     * @return This returns the coefficient for the feedforward factor.
     */
    double getConstant(double input);
}