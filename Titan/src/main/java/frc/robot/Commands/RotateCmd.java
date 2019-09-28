package frc.robot.Commands;

import frc.robot.Enums.AutoObjective;

public class RotateCmd extends BaseCmd {
    public RotateCmd(double theta) {
        super(AutoObjective.DRIVE_ROTATE, theta);
    }
}