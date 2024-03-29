package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Enums.AutoObjective;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class Intake extends Mechanism {
    private static Intake instance = new Intake();
    private VictorSPX leftArm, rightArm;
    //private TalonSRX tiltTalon;
    private CANSparkMax tiltSpark;
    private CANPIDController tiltPID;
    //private PIDController pid;
    private CANEncoder tiltEncoder;
    private DigitalInput cargoButton;
    private DoubleSolenoid leftHatchPistons, rightHatchPistons, hatchGrabber;
    private Mode autoMode;
    private enum Mode {SETUP, WAITUNTILGRABRELEASED, WAITUNTILFINISHED};
    private long startTime, timeout;
    private boolean cargoButtonPressed, taskCompleted, firstTime;

    public Intake() {
        super();
        leftArm = new VictorSPX(Constants.LEFT_INTAKE_PORT);
        rightArm = new VictorSPX(Constants.RIGHT_INTAKE_PORT);
        leftArm.follow(rightArm);
        leftArm.setNeutralMode(NeutralMode.Brake);
        rightArm.setNeutralMode(NeutralMode.Brake);
        //tiltTalon = new TalonSRX(Constants.TILT_MOTOR_PORT);
        //tiltTalon.config_kP(0, 0.3);
        //tiltTalon.config_kI(0, 0.0);
        //tiltTalon.config_kD(0, 0.0);
        //tiltTalon.setInverted(false);
        tiltSpark = new CANSparkMax(Constants.TILT_MOTOR_PORT, MotorType.kBrushless);
        tiltPID = new CANPIDController(tiltSpark);
        //pid = new PIDController();
        tiltEncoder = tiltSpark.getEncoder();
        cargoButton = new DigitalInput(Constants.CARGO_BUTTON_PORT);
        tiltSpark.setIdleMode(IdleMode.kBrake);

        //leftHatchPistons = new DoubleSolenoid(0, Constants.LEFT_HATCH_SOLENOID_REVERSE, Constants.RIGHT_HATCH_SOLENOID_FORWARD);
        //rightHatchPistons = new DoubleSolenoid(0, Constants.RIGHT_HATCH_SOLENOID_REVERSE, Constants.LEFT_HATCH_SOLENOID_FORWARD);
        leftHatchPistons = new DoubleSolenoid(0, Constants.RIGHT_HATCH_SOLENOID_FORWARD,  Constants.RIGHT_HATCH_SOLENOID_REVERSE);
        rightHatchPistons = new DoubleSolenoid(0, Constants.LEFT_HATCH_SOLENOID_REVERSE, Constants.LEFT_HATCH_SOLENOID_FORWARD);
        hatchGrabber = new DoubleSolenoid(0,  Constants.HATCH_GRABBER_SOLENOID_FORWARD, Constants.HATCH_GRABBER_SOLENOID_REVERSE);



        //leftHatchPistons = new DoubleSolenoid(0,Constants.LEFT_HATCH_SOLENOID_FORWARD, Constants.LEFT_HATCH_SOLENOID_REVERSE);
        //rightHatchPistons = new DoubleSolenoid(0,Constants.RIGHT_HATCH_SOLENOID_FORWARD, Constants.RIGHT_HATCH_SOLENOID_REVERSE);

        taskCompleted = true;
        firstTime = true;
        cargoButtonPressed = false;
        autoMode = Mode.SETUP;
        startTime = 0;

        leftArm.setInverted(true);

        //pid.start(Constants.TILT_GAINS);
        tiltPID.setP(Constants.TILT_GAINS[0]);
        tiltPID.setI(Constants.TILT_GAINS[1]);
        tiltPID.setD(Constants.TILT_GAINS[2]);
        tiltPID.setIZone(Constants.TILT_GAINS[3]);
        tiltPID.setFF(Constants.TILT_GAINS[4]);
        tiltPID.setOutputRange(-1.0, 1.0);
        tiltEncoder.setPosition(0.0);
    }

    public void drive() {
        if(super.size() > 0 || (target != null && !taskCompleted)) {
            if(super.size() > 0 && (target == null || taskCompleted)) {
                target = super.pop();
                timeout = System.currentTimeMillis();
            }
            if(super.peek() == null) {
                super.pop();
            }

            switch(target.getObjective()) {
                case TILTCARGO:
                    //changeIntakeMode(IntakePosition.CARGO);
                    break;
                case TILTHATCH:
                    //changeIntakeMode(IntakePosition.HATCH);
                    break;
                case TILTTOPROCKET:
                    //changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
                    break;
                case SHOOTCARGO:
                    autoShootCargo();
                    break;
                case COLLECTCARGO:
                    autoCollectCargo();
                    break;
                case SHOOTHATCH:
                    autoShootHatch();
                    break;
                default:
                    break;
            }
        }
    }

    public void drive(XboxController controller) {
        //System.out.println(tiltEncoder.getPosition());
        if(firstTime) {
            super.flush();
            firstTime = false;
            hatchGrabber.set(Value.kForward);
            rightHatchPistons.set(Value.kForward
            );
            leftHatchPistons.set(Value.kReverse);
        }

        if(hatchGrabber.get() == Value.kForward) {
            (new JoystickRumble(controller, 1, 0.1)).start();;
        }

        
        if(controller.getStartButtonPressed()) {
            super.flush();
        }
        
        if(controller.getBumper(Hand.kRight) /*&& (Elevator.getInstance().getIntakePosition() == IntakePosition.CARGO || Elevator.getInstance().getIntakePosition() == IntakePosition.TOP_ROCKET_TILT)*/) {
            rightArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
            leftArm.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
        } 
        else if(controller.getBumper(Hand.kLeft) /*&& (Elevator.getInstance().getIntakePosition() == IntakePosition.CARGO || Elevator.getInstance().getIntakePosition() == IntakePosition.TOP_ROCKET_TILT)*/) {
            rightArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
            leftArm.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
        } 
        else {
            rightArm.set(ControlMode.PercentOutput, 0);
            leftArm.set(ControlMode.PercentOutput, 0);
        }

        if(controller.getBButtonReleased()) {
            super.push(new IntakeCmd(AutoObjective.SHOOTHATCH, -1));
        }

        if(/*taskCompleted && */controller.getYButton()) {
            if(Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.5) == 0)
                tiltSpark.set(calculateTiltVelocity(true));
            else
                tiltSpark.set(0.65);
            //tiltTalon.set(ControlMode.PercentOutput, 0.5);
        } else if(/*taskCompleted && */controller.getXButton()) {
            if(Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.5) == 0)
                tiltSpark.set(calculateTiltVelocity(false));
            else
                tiltSpark.set(-0.23);
            //tiltTalon.set(ControlMode.PercentOutput, -0.5);
        }
        else /*if(taskCompleted)*/ {
            //tiltTalon.set(ControlMode.PercentOutput,0.0);
            tiltSpark.set(0.0);
        }
        
        if(controller.getAButtonPressed()) {
            if(rightHatchPistons.get() == Value.kForward) {
                rightHatchPistons.set(Value.kReverse);
            } else {
                rightHatchPistons.set(Value.kForward);
            }
        }
        
        if(controller.getStickButtonReleased(Hand.kRight)) {
            if(hatchGrabber.get() == Value.kForward) {
                hatchGrabber.set(Value.kReverse);
            } else {
                hatchGrabber.set(Value.kForward);
            }
        }
        
        if(controller.getStickButton(Hand.kLeft))
         {
            tiltEncoder.setPosition(0);
        }

        drive();
    }

    public double calculateTiltVelocity(boolean commandedUp) {
        if(commandedUp) {
            return 0.5 * Math.sin(tiltEncoder.getPosition() * (Math.PI/10)) + 0.1;
        } else {
            return -0.3 * Math.sin(tiltEncoder.getPosition() / 10) - 0.1;
        }
    } 
    
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

    public void autoShootHatch() {
        switch(autoMode) {
            case SETUP:
                taskCompleted = false;
                startTime = System.currentTimeMillis();
                if(hatchGrabber.get() == Value.kForward)
                    hatchGrabber.set(Value.kReverse);
                SWDrive.getInstance().rumbleController(0.75);
                autoMode = Mode.WAITUNTILGRABRELEASED;
                break;
            case WAITUNTILGRABRELEASED:
                if(System.currentTimeMillis() - startTime >= 200) {
                    leftHatchPistons.set(Value.kForward);
                    autoMode = Mode.WAITUNTILFINISHED;
                }
                break;
            case WAITUNTILFINISHED:
                if(System.currentTimeMillis() - startTime >= 750) {
                    leftHatchPistons.set(Value.kReverse);
                    //rightHatchPistons.set(Value.kReverse);
                    autoMode = Mode.SETUP;
                    taskCompleted = true;
                    return;
                }
                break;
        }
    }
    public void changeIntakeMode(IntakePosition targetPosition) {
        if(taskCompleted) {
            taskCompleted = false;
        }
        if(targetPosition != Elevator.getInstance().getIntakePosition()) {
            //This shouldn't ever be called, intake tilt is now completely manual
            //tiltPID.setReference(targetPosition.getTargetEncValue(), ControlType.kPosition); 
        }
        if(atTarget(targetPosition)) {
            Elevator.getInstance().setIntakePosition(targetPosition);
            Elevator.getInstance().updateTalonPIDProfile();
            taskCompleted = true;
            return;
        }
        if(System.currentTimeMillis() - timeout >= 1000) {
            System.out.println("TIMED OUT");
            Elevator.getInstance().setIntakePosition(targetPosition);
            taskCompleted = true;
            return;
        }
    }

    //Wrapper Function to check if at Target
    private boolean atTarget(IntakePosition targetPosition) {
        return checkEncoderValueInRange(targetPosition);
    }

    private boolean checkEncoderValueInRange(IntakePosition desiredPosition) {
        return Math.abs(desiredPosition.getTargetEncValue() - tiltEncoder.getPosition()) <= Constants.TILT_THRESHOLD;
    }
    
    public static Intake getInstance() {
        return instance;
    }

    public void onDemandTest() {
        //TODO: Fix this
        /*double prevEncoderCount = tiltSpark.getEncoder().getPosition();
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
            System.out.println("   DIFF:" + tiltSpark.getEncoder().getPosition());*/
        //}
    }
}