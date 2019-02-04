package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Enums.IntakePosition;

public class Intake extends Mechanism {
    private static Intake instance = new Intake();
    private TalonSRX leftArm, rightArm;
    private CANSparkMax tiltMotor;
    private CANPIDController tiltPID;
    private DoubleSolenoid hatchPusher;
    private DigitalInput cargoButton;

    private boolean taskCompleted;

    public Intake() {
        super();
        leftArm = new TalonSRX(Constants.LEFT_INTAKE_PORT);
        rightArm = new TalonSRX(Constants.RIGHT_INTAKE_PORT);
        leftArm.follow(rightArm);
        tiltMotor = new CANSparkMax(Constants.TILT_MOTOR_PORT, MotorType.kBrushless);
        tiltPID = new CANPIDController(tiltMotor);
        hatchPusher = new DoubleSolenoid(Constants.HATCH_SOLENOID_FORWARD, Constants.HATCH_SOLENOID_REVERSE);
        cargoButton = new DigitalInput(Constants.CARGO_BUTTON_PORT);

        taskCompleted = true;

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
        if(cargoButton.get() || !controller.getBButton()) {
            rightArm.set(ControlMode.PercentOutput, 0);
        } else if(controller.getAButton()) {
            rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
        } else if(controller.getBButton()) {
            rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
        } else {
            rightArm.set(ControlMode.PercentOutput, 0);
        }

        if(controller.getBumper(Hand.kLeft)) {
            changeIntakeMode(IntakePosition.HATCH);
        } else if(controller.getBumper(Hand.kRight)) {
            changeIntakeMode(IntakePosition.CARGO);
        } else if(controller.getXButton()) {
            changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
        }else {
        drive();
        }
    }

    public void autoShootCargo() {
        for(int i=0;i<5;i++) {
            rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
            Timer.delay(0.2); //TODO: Adjust Time
        }
        rightArm.set(ControlMode.PercentOutput, 0);
    }

    public void autoCollectCargo() {
        for(int i=0;i<5;i++) {
            rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
            Timer.delay(0.2);
        }
        rightArm.set(ControlMode.PercentOutput, 0);
    }

    public void autoShoothHatch() {
        if(hatchPusher.get() == Value.kReverse)
            hatchPusher.set(Value.kForward);
        Timer.delay(2);
        hatchPusher.set(Value.kReverse);
    }

    public void changeIntakeMode(IntakePosition targetPos) {
        tiltPID.setReference(targetPos.getTargetEncValue(), ControlType.kPosition);
    }
    
    public static Intake getInstance() {
        return instance;
    }
}