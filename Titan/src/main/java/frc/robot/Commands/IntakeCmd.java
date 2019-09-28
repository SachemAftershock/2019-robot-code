package frc.robot.Commands;

import frc.robot.Enums.AutoObjective;

public class IntakeCmd extends BaseCmd {
    public IntakeCmd(AutoObjective objective, double setpoint) {
        super(objective, setpoint);
    }

    public IntakeCmd(AutoObjective objective) {
        super(objective);
    }
}