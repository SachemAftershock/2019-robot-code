package frc.robot;

class RotateCmd extends BaseCmd {
    public RotateCmd(double theta) {
        super(AutoObjective.DRIVEROTATE, theta);
    }
}