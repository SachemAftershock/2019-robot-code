package frc.robot.Enums;

public enum Rotation {
    CLOCKWISE(1), COUNTERCLOCKWISE(-1);

    private int direction;
    Rotation(int direction) {
        this.direction = direction;
    }

    public double getConversionFactor() {
        return direction;
    }
}