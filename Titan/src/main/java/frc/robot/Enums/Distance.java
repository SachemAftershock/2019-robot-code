package frc.robot.Enums;

import frc.robot.Constants;

public enum Distance {
    INCHES(1.0), METERS(39.37), FEET(12.0);

    private double conversionFactor;
    Distance(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    public double getConversionFactor() {
        return conversionFactor / Constants.WHEEL_CIRCUMFERENCE * Constants.ENCODER_TICKS_PER_ROT;
    }
}