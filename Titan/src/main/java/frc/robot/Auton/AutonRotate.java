package frc.robot.Auton;

import frc.robot.Enums.Rotation;

public class AutonRotate extends AutonBase {
    public AutonRotate(Rotation rotate, double setpoint) {
        super(AutonTask.LINEAR, setpoint);
        setpoint *= rotate.getConversionFactor();
    }
}