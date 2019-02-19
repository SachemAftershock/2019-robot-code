package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class SimpleElevator {
    private TalonSRX talon;
    private static SimpleElevator instance = new SimpleElevator();

    public SimpleElevator() {
        talon = new TalonSRX(4);
    }

    public void drive(XboxController controller) {
        talon.set(ControlMode.PercentOutput, controller.getY(Hand.kLeft));
    }

    public static SimpleElevator getInstance() {
        return instance;
    }
}