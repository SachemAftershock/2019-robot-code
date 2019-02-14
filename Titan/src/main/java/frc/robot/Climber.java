package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commands.ClimbCmd;
import frc.robot.Commands.DriveCmd;
import frc.robot.Commands.DriveCmd.Distance;
import frc.robot.Enums.AutoObjective;
import frc.robot.Enums.IntakePosition;


public class Climber extends Mechanism {
    private static Climber instance = new Climber();
    private CANSparkMax backSlave, backMaster, frontSlave, frontMaster;
    private TalonSRX climbWheelTalon;
    private DigitalInput topBackLS, topFrontLS, bottomBackLS, bottomFrontLS;
    private boolean setpointReached;
    private float relativeSpeedAdjust;
    private DriveForwardState driveState;
    private LegState legState;
    private long startTime;
    enum Legs {FRONT, BACK, BOTH};
    enum DriveForwardState {STARTTIMER, WAITFORTIMEREXPIRATION};
    enum LegState {STARTLEGS, WAITFORLIMITS};

    


    public Climber() {
        backSlave = new CANSparkMax(Constants.CLIMBER_BACK_LEFT_SPARK_PORT, MotorType.kBrushless);
        backMaster = new CANSparkMax(Constants.CLIMBER_BACK_RIGHT_SPARK_PORT, MotorType.kBrushless);
        frontSlave = new CANSparkMax(Constants.CLIMBER_FRONT_LEFT_SPARK_PORT, MotorType.kBrushless);
        frontMaster = new CANSparkMax(Constants.CLIMBER_FRONT_RIGHT_SPARK_PORT, MotorType.kBrushless);

        backSlave.follow(backMaster);
        frontSlave.follow(frontMaster);

        climbWheelTalon = new TalonSRX(Constants.CLIMB_BOTTOM_WHEELS_TALON);

        topBackLS = new DigitalInput(Constants.CLIMBER_TOP_BACK_LS);
        topFrontLS = new DigitalInput(Constants.CLIMBER_TOP_FRONT_LS);
        bottomBackLS = new DigitalInput(Constants.CLIMBER_BOTTOM_BACK_LS);
        bottomFrontLS = new DigitalInput(Constants.CLIMBER_BOTTOM_FRONT_LS);
        setpointReached = true;

        driveState = DriveForwardState.STARTTIMER;
        legState = LegState.STARTLEGS;

        relativeSpeedAdjust = 0;
        startTime = 0;
    }
    
    public void drive() {
        if(super.size() > 0 || !setpointReached) {
            if(target == null || setpointReached) {
                target = super.pop();
                relativeSpeedAdjust = 0;
            }
            
            switch(target.getObjective()) {
                case CLIMBEREXTENDBOTH:
                    climbExtend(Legs.BOTH);
                    break;
                case CLIMBEREXTENDFRONTLEGS:
                    climbExtend(Legs.FRONT);
                    break;
                case CLIMBEREXTENDBACKLEGS:
                    climbExtend(Legs.BACK);
                    break;
                case CLIMBERDRIVE:
                    climbDrive();
                    break;
                case RETRACTFRONTLEG:
                    retractLeg(Legs.FRONT);
                    break;
                case RETRACTBACKLEG:
                    retractLeg(Legs.BACK);
                    break;
                default:
                    break;
            }
        }
    }
    //TODO: Manual Mode
    public void climbExtend(Legs legs) {
        float thePitch = SWDrive.getInstance().getPitch();
        if(Math.abs(thePitch) > Constants.PITCH_LIMIT) {
            //TODO:Light Up Red LIGHT
        }
        switch(legState) {
            case STARTLEGS:
                setpointReached = false;
                if(legs == Legs.BACK){
                    backMaster.set(Constants.EXTEND_SPEED);
                } else if(legs == Legs.FRONT) {
                    frontMaster.set(Constants.EXTEND_SPEED);
                } else if (legs == Legs.BOTH) {
                    relativeSpeedAdjust = thePitch * Constants.PITCH_TO_SPEED_MODIFIER;
                    backMaster.set(Constants.EXTEND_SPEED + relativeSpeedAdjust);
                    frontMaster.set(Constants.EXTEND_SPEED - relativeSpeedAdjust);
                }
                legState = LegState.WAITFORLIMITS;
                break;

            case WAITFORLIMITS:
                if(legs == Legs.BACK) {
                    if(topBackLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                } else if(legs == Legs.FRONT) {
                    if(topFrontLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                } else if(legs == Legs.BOTH) {
                    if(topBackLS.get()) {
                        backMaster.set(0.0);
                    }
                    if(topFrontLS.get()) {
                        frontMaster.set(0.0);
                    }
                    if(topBackLS.get() && topFrontLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                }
                break;
        }
    }

    public void climbDrive() {
        switch(driveState) {
            case STARTTIMER:
                setpointReached = false;
                climbWheelTalon.set(ControlMode.PercentOutput, Constants.CLIMBER_DRIVE_SPEED);
                startTime = System.currentTimeMillis();
                driveState = DriveForwardState.WAITFORTIMEREXPIRATION;
                break;

            case WAITFORTIMEREXPIRATION:
                if(System.currentTimeMillis() - startTime >= 5000) {
                    climbWheelTalon.set(ControlMode.PercentOutput, 0.0);
                    driveState = DriveForwardState.STARTTIMER;
                    setpointReached = true;
                }
                break;
        }
    }

    public void retractLeg(Legs legs) {
        float thePitch = SWDrive.getInstance().getPitch();
        if(Math.abs(thePitch) > Constants.PITCH_LIMIT) {
            //TODO:Light Up Red LIGHT
        }
        switch(legState) {
            case STARTLEGS:
                setpointReached = false;
                if(legs == Legs.BACK){
                    backMaster.set(-Constants.EXTEND_SPEED);
                } else if(legs == Legs.FRONT) {
                    frontMaster.set(-Constants.EXTEND_SPEED);
                } else if (legs == Legs.BOTH) {
                    relativeSpeedAdjust = thePitch * Constants.PITCH_TO_SPEED_MODIFIER;
                    backMaster.set(-Constants.EXTEND_SPEED + relativeSpeedAdjust);
                    frontMaster.set(-Constants.EXTEND_SPEED - relativeSpeedAdjust);
                }
                legState = LegState.WAITFORLIMITS;
                break;

            case WAITFORLIMITS:
                if(legs == Legs.BACK) {
                    if(bottomBackLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                } else if(legs == Legs.FRONT) {
                    if(bottomFrontLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                } else if(legs == Legs.BOTH) {
                    if(bottomBackLS.get()) {
                        backMaster.set(0.0);
                    }
                    if(bottomFrontLS.get()) {
                        frontMaster.set(0.0);
                    }
                    if(bottomBackLS.get() && bottomFrontLS.get()) {
                        legState = LegState.STARTLEGS;
                        setpointReached = true;
                    }
                }
                break;
        }
    }


  
    public void startClimberSequence() {
        //Setpoint is unused, that's why it's -1
        Intake.getInstance().changeIntakeMode(IntakePosition.HATCH);
        super.push(new ClimbCmd(AutoObjective.CLIMBEREXTENDBOTH, -1));
        super.push(new ClimbCmd(AutoObjective.CLIMBERDRIVE, -1));
        super.push(new ClimbCmd(AutoObjective.RETRACTFRONTLEG, -1));
        SWDrive.getInstance().driveForClimbSequence();
        super.push(new ClimbCmd(AutoObjective.RETRACTFRONTLEG, -1));
        super.push(new ClimbCmd(AutoObjective.RETRACTBACKLEG, -1));
    }
    public void startDriveOffHabSequence() {
        Intake.getInstance().changeIntakeMode(IntakePosition.HATCH);
        SWDrive.getInstance().push(new DriveCmd(Distance.INCHES, Constants.HAB_DRIVE_OFF_DISTANCE));
        super.push(new ClimbCmd(AutoObjective.CLIMBEREXTENDFRONTLEGS, -1));
        SWDrive.getInstance().push(new DriveCmd(Distance.INCHES, Constants.HAB_DRIVE_OFF_DISTANCE));
        super.push(new ClimbCmd(AutoObjective.RETRACTFRONTLEG, -1));
    }

    public static Climber getInstance() {
        return instance;
    }

    public void onDemandTest() {
        double prevFrontEncoderCount = frontMaster.getEncoder().getPosition();
        double prevBackEncoderCount = backMaster.getEncoder().getPosition();
        boolean frontHealthy = true, backHealthy = true;
        frontMaster.set(0.1);
        backMaster.set(0.1);
        Timer.delay(1.5);
        if(Math.abs(prevFrontEncoderCount - frontMaster.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + frontMaster.getEncoder().getPosition());
        }
        if(Math.abs(prevBackEncoderCount - backMaster.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + backMaster.getEncoder().getPosition());
        }
        frontMaster.set(-0.1);
        backMaster.set(-0.1);
        Timer.delay(1.5);
        if(Math.abs(prevFrontEncoderCount - frontMaster.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + frontMaster.getEncoder().getPosition());
        }
        if(Math.abs(prevBackEncoderCount - backMaster.getEncoder().getPosition()) < 10) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + backMaster.getEncoder().getPosition());
        }
        frontMaster.set(0);
        backMaster.set(0);
        frontMaster.set(0.1);
        backMaster.set(0.1);
        Timer.delay(1.5);
        if(Math.abs(prevFrontEncoderCount - frontMaster.getEncoder().getPosition()) == 0) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + frontMaster.getEncoder().getPosition());
        }
        if(Math.abs(prevBackEncoderCount - backMaster.getEncoder().getPosition()) == 0) {
            System.out.println("INTAKE TILT ERROR");
            System.out.println("   DIFF:" + backMaster.getEncoder().getPosition());
        }
    }
}