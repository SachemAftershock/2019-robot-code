package frc.robot.Auton;

import frc.robot.Enums.Distance;;

public class AutonDrive extends AutonBase {
    public AutonDrive(Distance distance, double setpoint) {
        super(AutonTask.LINEAR, setpoint);
        setpoint *= distance.getConversionFactor();
    }
}