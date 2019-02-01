package frc.robot;

class DriveCmd extends BaseCmd {
    
    public DriveCmd(Distance metric, double distance) {
        super(AutoObjective.DRIVEROTATE, distance);
        setpoint *= metric.getConversionFactor() / Constants.WHEEL_CIRCUMFERENCE * Constants.ENCODER_TICKS_PER_ROT;
    }

    enum Distance {
        INCHES(1.0), METERS(39.37), FEET(12.0);

        private double conversionFactor;
        Distance(double conversionFactor) {
            this.conversionFactor = conversionFactor;
        }

        public double getConversionFactor() {
            return conversionFactor;
        }
    }

}