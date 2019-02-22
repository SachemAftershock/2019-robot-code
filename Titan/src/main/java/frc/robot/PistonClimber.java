package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PistonClimber extends Mechanism {
    private static PistonClimber instance = new PistonClimber();
    private DoubleSolenoid frontPistons, rearPistons;

    private PistonClimber() {
        frontPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_FRONT_PISTON_FORWARD_PORT, Constants.CLIMBER_FRONT_PISTON_REVERSE_PORT);
        rearPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_REAR_PISTON_FORWARD_PORT, Constants.CLIMBER_REAR_PISTON_REVERSE_PORT);
        frontPistons.set(Value.kReverse);
        rearPistons.set(Value.kReverse);
    }

    public void drive() {
        //Not really anything to put here, doubt we're going to be autonomously using this thingy
    }

    public void toggleFrontPistons() {
        if(frontPistons.get() != Value.kForward) {
            frontPistons.set(Value.kForward);
        } else {
            frontPistons.set(Value.kReverse);
        }
    }
    public void toggleRearPiston() {
        if(rearPistons.get() != Value.kForward) {
            rearPistons.set(Value.kForward);
        } else {
            rearPistons.set(Value.kReverse);
        }
    }

    public static PistonClimber getInstance() {
        return instance;
    }
}