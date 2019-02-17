package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Enums.IntakePosition;

public class Intake extends Mechanism {
    private static Intake instance = new Intake();
    private TalonSRX leftArm, rightArm;
    private CANSparkMax tiltSpark;
    private CANPIDController tiltPID;
    private CANEncoder tiltEncoder;
    private DigitalInput cargoButton;
    private DoubleSolenoid leftHatchPistons, rightHatchPistons;
    private Mode autoMode;
    private enum Mode {SETUP, WAITUNTILFINISHED};
    private long startTime;

    private boolean taskCompleted;
    private boolean hatchPistonsEngaged;

    public Intake() {
        super();
        leftArm = new TalonSRX(Constants.LEFT_INTAKE_PORT);
        rightArm = new TalonSRX(Constants.RIGHT_INTAKE_PORT);
        leftArm.follow(rightArm);
        tiltSpark = new CANSparkMax(Constants.TILT_MOTOR_PORT, MotorType.kBrushless);
        tiltPID = new CANPIDController(tiltSpark);
        tiltEncoder = tiltSpark.getEncoder();
        cargoButton = new DigitalInput(Constants.CARGO_BUTTON_PORT);

        leftHatchPistons = new DoubleSolenoid(Constants.LEFT_HATCH_SOLENOID_FORWARD, Constants.LEFT_HATCH_SOLENOID_REVERSE);
        rightHatchPistons = new DoubleSolenoid(Constants.RIGHT_HATCH_SOLENOID_FORWARD, Constants.RIGHT_HATCH_SOLENOID_REVERSE);

        taskCompleted = true;
        hatchPistonsEngaged = false;
        autoMode = Mode.SETUP;
        startTime = 0;

        leftArm.setInverted(true);

        tiltPID.setP(Constants.TILT_GAINS[0]);
        tiltPID.setI(Constants.TILT_GAINS[1]);
        tiltPID.setD(Constants.TILT_GAINS[2]);
        tiltPID.setIZone(Constants.TILT_GAINS[3]);
        tiltPID.setFF(Constants.TILT_GAINS[4]);
        tiltPID.setOutputRange(-1.0, 1.0);
    }

    public void drive() {
        if(!taskCompleted || super.size() > 0) {
            if(target == null || taskCompleted) {
                target = super.pop();
            }

            switch(target.getObjective()) {
                case TILTCARGO:
                    changeIntakeMode(IntakePosition.CARGO);
                    break;
                case TILTHATCH:
                    changeIntakeMode(IntakePosition.HATCH);
                    break;
                case TILTTOPROCKET:
                    changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
                    break;
                case SHOOTCARGO:
                    autoShootCargo();
                    break;
                case COLLECTCARGO:
                    autoCollectCargo();
                    break;
                case SHOOTHATCH:
                    autoShoothHatch();
                    break;
                default:
                    break;
            }
        }
    }

    public void drive(XboxController controller) {
        if(controller.getStartButton()) {
            super.flush();
        }
        if(cargoButton.get() && !controller.getBButton()) {
            rightArm.set(ControlMode.PercentOutput, 0);
        } 
        if(controller.getAButton() && (Elevator.getInstance().getIntakePosition() == IntakePosition.CARGO || Elevator.getInstance().getIntakePosition() == IntakePosition.TOP_ROCKET_TILT)) {
            rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
        } else if(controller.getBButton() && (Elevator.getInstance().getIntakePosition() == IntakePosition.CARGO || Elevator.getInstance().getIntakePosition() == IntakePosition.TOP_ROCKET_TILT)) {
            rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
        } 
        else {
            rightArm.set(ControlMode.PercentOutput, 0);
        }
        //TODO: Double Check this below
        if(controller.getBButton() && Elevator.getInstance().getIntakePosition() == IntakePosition.HATCH) {
            leftHatchPistons.set(Value.kForward);
            rightHatchPistons.set(Value.kForward);
            System.out.println("Pistons Engaged");
            hatchPistonsEngaged = false;
        } else {
            if(hatchPistonsEngaged) {
                leftHatchPistons.set(Value.kReverse);
                rightHatchPistons.set(Value.kReverse);
                System.out.println("Pistons Reversed");
                hatchPistonsEngaged = true;
            }
        }

        drive();
    }
    //TODO: Make this non blocking
    public void autoShootCargo() {
        switch(autoMode) {
            case SETUP:
                taskCompleted = false;
                startTime = System.currentTimeMillis();
                rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
                autoMode = Mode.WAITUNTILFINISHED;
                break;
            case WAITUNTILFINISHED:
                if(System.currentTimeMillis() - startTime >= 5000) {
                    rightArm.set(ControlMode.PercentOutput, 0.0);
                    autoMode = Mode.SETUP;
                    taskCompleted = true;
                }
                break;
        }
    }

    public void autoCollectCargo() {
        switch(autoMode) {
            case SETUP:
                taskCompleted = false;
                startTime = System.currentTimeMillis();
                rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
                autoMode = Mode.WAITUNTILFINISHED;
                break;
            case WAITUNTILFINISHED:
                if(System.currentTimeMillis() - startTime >= 5000) {
                    rightArm.set(ControlMode.PercentOutput, 0.0);
                    autoMode = Mode.SETUP;
                    taskCompleted = true;
                }
                break;
        }
    }

    public void autoShoothHatch() {
        switch(autoMode) {
            case SETUP:
                taskCompleted = false;
                startTime = System.currentTimeMillis();
                //leftHatchPistons.set(Value.kForward);
                //rightHatchPistons.set(Value.kForward);
                autoMode = Mode.WAITUNTILFINISHED;
                break;
            case WAITUNTILFINISHED:
                if(System.currentTimeMillis() - startTime >= 100) {
                    //leftHatchPistons.set(Value.kReverse);
                    //rightHatchPistons.set(Value.kReverse);
                    autoMode = Mode.SETUP;
                    taskCompleted = true;
                }
                break;
        }
    }

    public void changeIntakeMode(IntakePosition targetPosition) {
        taskCompleted = false;
        if(targetPosition != Elevator.getInstance().getIntakePosition())
            tiltPID.setReference(targetPosition.getTargetEncValue(), ControlType.kPosition); 
        Elevator.getInstance().setIntakePosition(targetPosition);
        Elevator.getInstance().updateTalonPIDProfile();
        if(atTarget(targetPosition)) {
            taskCompleted = true;
        }
    }

    private boolean atTarget(IntakePosition targetPosition) {
        return checkEncoderValueInRange(targetPosition);
    }

    private boolean checkEncoderValueInRange(IntakePosition desiredPosition) {
        return tiltEncoder.getPosition() >= desiredPosition.getTargetEncValue() - Constants.INTAKE_TILT_ENC_THRESHOLD &&
            tiltEncoder.getPosition() <= desiredPosition.getTargetEncValue() + Constants.INTAKE_TILT_ENC_THRESHOLD;
    }
    
    public static Intake getInstance() {
        System.out.println("-------Intake::getInstake");
        return instance;
    }

    public void onDemandTest() {
        double prevEncoderCount = tiltSpark.getEncoder().getPosition();
        rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
        Timer.delay(2);
        rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
        Timer.delay(2);
        rightArm.set(ControlMode.PercentOutput, 0.0);
        changeIntakeMode(IntakePosition.CARGO);
        Timer.delay(5);
        if(Math.abs(prevEncoderCount - tiltSpark.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + tiltSpark.getEncoder().getPosition());
        }
        prevEncoderCount = tiltSpark.getEncoder().getPosition();
        changeIntakeMode(IntakePosition.HATCH);
        Timer.delay(5);
        if(Math.abs(prevEncoderCount - tiltSpark.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + tiltSpark.getEncoder().getPosition());
        }
        prevEncoderCount = tiltSpark.getEncoder().getPosition();
        changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
        Timer.delay(5);
        if(Math.abs(prevEncoderCount - tiltSpark.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + tiltSpark.getEncoder().getPosition());
        }
        prevEncoderCount = tiltSpark.getEncoder().getPosition();
        changeIntakeMode(IntakePosition.HATCH);
        Timer.delay(5);
        if(Math.abs(prevEncoderCount - tiltSpark.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + tiltSpark.getEncoder().getPosition());
        }
    }
}