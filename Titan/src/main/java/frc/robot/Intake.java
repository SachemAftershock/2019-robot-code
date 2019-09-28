package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Enums.IntakePosition;

/**
 * Subsystem for Arm Intake Mechanism
 * 
 * @author Shreyas Prasad
 */
public class Intake {
    private static Intake instance = new Intake();
    private VictorSPX leftArm, rightArm;
    private CANSparkMax wristSpark;
    private CANPIDController wristPID;
    private CANEncoder wristEncoder;
    private DoubleSolenoid hatchShooter, armExpander, hatchGrabber;
    private Mode autoMode;
    private enum Mode {SETUP, WAITUNTILGRABRELEASED, STARTHATCHSHOOT, WAITUNTILFINISHED};
    private enum Direction {UP, DOWN}
    private Timer timer;

    /**
     * Constructor for Intake Class
     */
    private Intake() {
        leftArm = new VictorSPX(Constants.LEFT_INTAKE_PORT);
        leftArm.setNeutralMode(NeutralMode.Brake);
        leftArm.setInverted(true);

        rightArm = new VictorSPX(Constants.RIGHT_INTAKE_PORT);
        rightArm.setNeutralMode(NeutralMode.Brake);
        rightArm.setInverted(false);

        wristSpark = new CANSparkMax(Constants.WRIST_MOTOR_PORT, MotorType.kBrushless);
        wristPID = wristSpark.getPIDController();
        wristEncoder = wristSpark.getEncoder();
        wristSpark.setIdleMode(IdleMode.kBrake);

        //TODO: Tune all the values below, they were pulled from a random example
        wristPID.setP(Constants.WRIST_PID[0]);
        wristPID.setI(Constants.WRIST_PID[1]);
        wristPID.setD(Constants.WRIST_PID[2]);
        wristPID.setIZone(Constants.WRIST_PID[3]);
        wristPID.setFF(Constants.WRIST_PID[4]);
        wristPID.setOutputRange(-1, 1);

        wristPID.setSmartMotionMaxVelocity(Constants.WRIST_MAX_VEL, Constants.WRIST_SMART_MOTION_SLOT);
        wristPID.setSmartMotionMinOutputVelocity(0, Constants.WRIST_SMART_MOTION_SLOT); //TODO: Find out what this should b
        wristPID.setSmartMotionMaxAccel(Constants.WRIST_MAX_ACC, Constants.WRIST_SMART_MOTION_SLOT);
        wristPID.setSmartMotionAllowedClosedLoopError(Constants.INTAKE_WRIST_EPSILON, Constants.WRIST_SMART_MOTION_SLOT);



        hatchShooter = new DoubleSolenoid(Constants.INTAKE_PCM_PORT, Constants.HATCH_SHOOTER_SOLENOID_FORWARD,  Constants.HATCH_SHOOTER_SOLENOID_REVERSE);
        armExpander = new DoubleSolenoid(Constants.INTAKE_PCM_PORT, Constants.ARM_EXPAND_SOLENOID_FORWARD, Constants.ARM_EXPAND_SOLENOID_REVERSE);
        hatchGrabber = new DoubleSolenoid(Constants.INTAKE_PCM_PORT,  Constants.HATCH_GRABBER_SOLENOID_FORWARD, Constants.HATCH_GRABBER_SOLENOID_REVERSE);
        autoMode = Mode.SETUP;
        timer = new Timer();
    }

    /**
     * Calculates Speed for Wrist Tilt Based on Planned Sinusoidal Functions
     * 
     * @param direction Selects function for upwards tilt path or downwards tilt path
     * 
     * @return Velocity calculated for wrist tilt from selected function
     */
    public double calculateTiltVelocity(Direction direction) {
        if(direction == Direction.UP) {
            return 0.5 * Math.sin(wristEncoder.getPosition() * (Math.PI/10)) + 0.1;
        } else {
            return -0.3 * Math.sin(wristEncoder.getPosition() / 10) - 0.1;
        }
    } 

    /**
     * Shoots Pistons to launch hatch off of velcro after
     * releasing hatch grabbing mechanism
     */
    public void shootHatch() {
        switch(autoMode) {
            case SETUP:
                Superstructure.getInstance().setSetpointReached(false);
                timer.start();
                if(hatchGrabber.get() != Value.kReverse)
                    hatchGrabber.set(Value.kReverse);
                SWDrive.getInstance().rumbleController(0.7);
                autoMode = Mode.WAITUNTILGRABRELEASED;
                break;

            case WAITUNTILGRABRELEASED:
                //TODO: Test if the correct method should be this or doing .get() to check elapsed time
                // or go back to System.currentTimeMillis()
                if(timer.hasPeriodPassed(.2)) {
                    autoMode = Mode.STARTHATCHSHOOT;
                }
                break;
            
            case STARTHATCHSHOOT:
                hatchShooter.set(Value.kForward);
                autoMode = Mode.WAITUNTILFINISHED;
                break;

            case WAITUNTILFINISHED:
                if(timer.hasPeriodPassed(.75)) {
                    hatchShooter.set(Value.kReverse);
                    autoMode = Mode.SETUP;
                    timer.stop();
                    timer.reset();
                    Superstructure.getInstance().setSetpointReached(true);
                    return;
                }
                break;
        }
    }

    public void shootCargo() {
        leftArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
        rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
    }
    public void intakeCargo() {
        leftArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
        rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
    }

    public void stopIntake() {
        leftArm.set(ControlMode.PercentOutput, 0.0);
        rightArm.set(ControlMode.PercentOutput, 0.0);
    }

    public void openArms() {
        armExpander.set(Value.kForward);
    }
    public void closeArms() {
        armExpander.set(Value.kReverse);
    }

    public void activateHatchGrab() {
        hatchGrabber.set(Value.kForward);
    }
    public void releaseHatchGrab() {
        hatchGrabber.set(Value.kReverse);
    }

    public void retractHatchShooter() {
        hatchShooter.set(Value.kReverse);
    }

    public void zeroWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public void tiltWristUp(boolean ignoreEncoder) {
        if(ignoreEncoder) {
            wristSpark.set(1.0);
        } else {
            wristSpark.set(calculateTiltVelocity(Direction.UP));
        }
    }

    public void tiltWristDown(boolean ignoreEncoder) {
        if(ignoreEncoder) {
            wristSpark.set(-1);
        } else {
            wristSpark.set(calculateTiltVelocity(Direction.DOWN));
        }
    }

    public void stopWristTilt() {
        wristSpark.set(0);
    }

    public void setWristSpeed(double speed) {
        wristSpark.set(speed);
    }

    public void tiltWristToPosition(IntakePosition targetPosition) {
        if(Superstructure.getInstance().getSetpointReached()) {
            Superstructure.getInstance().setSetpointReached(false);
        }
        if(!Utilities.withinEpsilon(targetPosition.getTargetEncValue(), wristEncoder.getPosition(), Constants.INTAKE_WRIST_EPSILON)) {
            wristPID.setReference(targetPosition.getTargetEncValue(), ControlType.kSmartMotion);
        } else {
            Superstructure.getInstance().setSetpointReached(true);
        }
    }

    //TODO: Find out if this is okay for me too do, it should be tbh
    public IntakePosition getIntakePosition() {
        if(Utilities.withinEpsilon(IntakePosition.HATCH.getTargetEncValue(), wristEncoder.getPosition(), Constants.INTAKE_WRIST_EPSILON)) {
            return IntakePosition.HATCH;
        } else {
            return IntakePosition.CARGO;
        }
    }
    
    /**
     * Gets intance of Intake singleton
     * 
     * @return Instance of Intake
     */
    public static Intake getInstance() {
        return instance;
    }
}