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
    enum Legs {FRONT, BACK, BOTH};


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
    }

    public void drive() {
        if(super.size() > 0 || !setpointReached) {
            if(target == null || setpointReached) {
                target = super.pop();
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

    public void climbExtend(Legs legs) {
        setpointReached = false;
        if(!topBackLS.get() && !topFrontLS.get()) {
            if(legs == Legs.BACK)
                backMaster.set(Constants.EXTEND_SPEED);
            else if(legs == Legs.FRONT)
                frontMaster.set(Constants.EXTEND_SPEED);
            else if(legs == Legs.BOTH) {
                backMaster.set(Constants.EXTEND_SPEED);
                frontMaster.set(Constants.EXTEND_SPEED);
            }
        } else {
            backMaster.set(0.0);
            frontMaster.set(0.0);
            setpointReached = true;
            return;
        }
        climbExtend(legs);
    }

    public void climbDrive() {
        setpointReached = false;
        for(int i=0;i<5;i++) {
            climbWheelTalon.set(ControlMode.PercentOutput, Constants.CLIMBER_DRIVE_SPEED);
            Timer.delay(1);
        }
        climbWheelTalon.set(ControlMode.PercentOutput, 0.0);
        setpointReached = true;
    }

    public void retractLeg(Legs legs) {
        setpointReached = false;
        if(legs == Legs.FRONT) {
            frontMaster.set(-Constants.EXTEND_SPEED);
            if(bottomFrontLS.get()) {
                frontMaster.set(0.0);
                setpointReached = true;
                return;
            }
        } else if (legs == Legs.BACK) {
            backMaster.set(-Constants.EXTEND_SPEED);
            if(bottomBackLS.get()) {
                backMaster.set(0.0);
                setpointReached = true;
                return;
            }
        } else if(legs == Legs.BOTH) {
            backMaster.set(-Constants.EXTEND_SPEED);
            frontMaster.set(-Constants.EXTEND_SPEED);
            if(bottomBackLS.get())
                backMaster.set(0.0);
            if(bottomFrontLS.get())
                frontMaster.set(0.0);
            if(bottomBackLS.get() && bottomFrontLS.get()) {
                setpointReached = true;
                return;
            }
        }
        retractLeg(legs);
    }


    //I don't feel like moving the XboxController to be a object in Robot
    //and rewriting part of Rohan's drivebase, so this function will be called
    //in SW Drive to add to the command queue, super wack, I know
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

    public boolean onDemandTest() {
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
        return frontHealthy && backHealthy;
    }
}