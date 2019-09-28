package frc.robot.Commands;

import frc.robot.Enums.AutoObjective;

public class CargoSearchCmd extends BaseCmd {
    public CargoSearchCmd(double direction) {
        super(AutoObjective.CARGO_SEARCH, direction);
    }
}