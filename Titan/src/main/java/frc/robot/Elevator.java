package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class Elevator extends Mechanism {
    private static Elevator instance = new Elevator();
    private IntakePosition intakeMode;
    private ElevatorPosition elevatorPosition;
    private ElevatorPosition targetPosition;
    private TalonSRX elevatorTalon;
    private DigitalInput topLS, bottomLS;
    private Joystick buttonBox;
    private LIDAR lidar;
    private int[] buttonID;
    private ElevatorPosition[] levels;
    private boolean encoderHealthy;
    private boolean completeManualOverride;
    private int encCount;
    private double lidarValue;
    private boolean lsActive;

    private Map<Integer, ElevatorInfo> elevatorMap;

    public static Elevator getInstance() {
        return instance;
    }
    
    public Elevator() {
        super();
        intakeMode = IntakePosition.HATCH;
        buttonBox = new Joystick(Constants.BUTTON_BOX_PORT);
        lidar = new LIDAR(new DigitalInput(Constants.ELEVATOR_LIDAR_PORT));
        elevatorTalon = new TalonSRX(Constants.ELEVATOR_TALON_PORT);
        topLS = new DigitalInput(Constants.TOP_LS_PORT);
        bottomLS = new DigitalInput(Constants.BOTTOM_LS_PORT);
        elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);
        elevatorTalon.config_kP(1, Constants.ELEVATOR_GAINS[0], 0);
        elevatorTalon.config_kI(1, Constants.ELEVATOR_GAINS[1], 0);
        elevatorTalon.config_kD(1, Constants.ELEVATOR_GAINS[2], 0);
        elevatorTalon.config_kF(1, Constants.ELEVATOR_GAINS[3], 0);
        elevatorTalon.setInverted(false);
        elevatorTalon.configMotionAcceleration(1000, 0);
        elevatorTalon.configMotionCruiseVelocity(5000, 0);
        elevatorTalon.config_IntegralZone(1, 200, 0);
        elevatorTalon.configClosedloopRamp(1, 256);
        elevatorTalon.configOpenloopRamp(1, 256);
        elevatorTalon.configAllowableClosedloopError(1, Constants.ENC_THRESHOLD, 0);

        encCount = elevatorTalon.getSelectedSensorPosition(1);
        lidarValue = lidar.getDistanceCm();
        encoderHealthy = true;
        completeManualOverride = false;
        lsActive = true;
        buttonID = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
        levels = new ElevatorPosition[] {ElevatorPosition.FLOOR, ElevatorPosition.LOW, ElevatorPosition.MID, ElevatorPosition.HIGH};

        elevatorMap.put(1, new ElevatorInfo(ElevatorPosition.FLOOR, null, Constants.FLOOR_TARGET_AZIMUTH));
        
        elevatorMap.put(2, new ElevatorInfo(ElevatorPosition.LOW, IntakePosition.HATCH, Constants.LEFT_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(3, new ElevatorInfo(ElevatorPosition.LOW, IntakePosition.CARGO, Constants.MID_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(4, new ElevatorInfo(ElevatorPosition.LOW, IntakePosition.HATCH, Constants.RIGHT_ROCKET_TARGET_AZIMUTH));

        elevatorMap.put(5, new ElevatorInfo(ElevatorPosition.MID, IntakePosition.HATCH, Constants.LEFT_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(6, new ElevatorInfo(ElevatorPosition.MID, IntakePosition.CARGO, Constants.MID_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(7, new ElevatorInfo(ElevatorPosition.MID, IntakePosition.HATCH, Constants.RIGHT_ROCKET_TARGET_AZIMUTH));

        elevatorMap.put(8, new ElevatorInfo(ElevatorPosition.HIGH, IntakePosition.HATCH, Constants.LEFT_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(9, new ElevatorInfo(ElevatorPosition.HIGH, IntakePosition.TOP_ROCKET_TILT, Constants.MID_ROCKET_TARGET_AZIMUTH));
        elevatorMap.put(10, new ElevatorInfo(ElevatorPosition.HIGH, IntakePosition.HATCH, Constants.RIGHT_ROCKET_TARGET_AZIMUTH));

        elevatorMap.put(11, new ElevatorInfo(ElevatorPosition.MID, null, Constants.LEFT_CARGO_TARGET_AZIMUTH));
        elevatorMap.put(12, new ElevatorInfo(ElevatorPosition.MID, null, Constants.MID_CARGO_TARGET_AZIMUTH));
        elevatorMap.put(13, new ElevatorInfo(ElevatorPosition.MID, null, Constants.RIGHT_CARGO_TARGET_AZIMUTH));
        
        elevatorMap.put(14, new ElevatorInfo(ElevatorPosition.MID, null, Constants.LOADING_STATION_TARGET_AZIMUTH));
    }
    public void drive() {
        if(topLS.get()) {
            elevatorPosition = ElevatorPosition.HIGH;
            encCount = ElevatorPosition.HIGH.getTargetEncValue();
        } if(bottomLS.get()) {
            elevatorPosition = ElevatorPosition.LOW;
            encCount = ElevatorPosition.LOW.getTargetEncValue();
        } if(!atTarget()) {
            commandElevator(targetPosition);
        } else {
            targetPosition = elevatorPosition;
        }

        for(int id : buttonID) {
            if(buttonBox.getRawButton(id)) 
                commandElevator(id);
        }

        if(super.size() > 0 || !atTarget()) {
            if(target == null || atTarget()) {
                target = super.pop();
            }
        
            switch(target.getObjective()) {
                case ELEVATORLOW:
                    commandElevator(ElevatorPosition.LOW);
                    break;
                case ELEVATORMID:
                    commandElevator(ElevatorPosition.MID);
                    break;
                case ELEVATORHIGH:
                    commandElevator(ElevatorPosition.HIGH);
                    break;
                default:
                    break;
            }
        }

        if(buttonBox.getRawButton(Constants.INTAKE_MODE_TOGGLE_BUTTON)) {
            if(intakeMode == IntakePosition.HATCH) {
                if(elevatorPosition == ElevatorPosition.HIGH) {
                    Intake.getInstance().changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
                } else {
                    Intake.getInstance().changeIntakeMode(IntakePosition.CARGO);
                }
                intakeMode = IntakePosition.CARGO;
            } else if(intakeMode == IntakePosition.CARGO || intakeMode == IntakePosition.TOP_ROCKET_TILT) {
                Intake.getInstance().changeIntakeMode(IntakePosition.HATCH);
            }
        }
        if(elevatorPosition == ElevatorPosition.HIGH && intakeMode == IntakePosition.CARGO) {
            Intake.getInstance().changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
        }
    }
    
    public void drive(XboxController controller) {
        encCount = elevatorTalon.getSelectedSensorPosition(0);
        lidarValue = lidar.getDistanceCm();
        if(controller.getBackButton()) {
            completeManualOverride = !completeManualOverride;
        } if(controller.getStartButton()) {
            super.flush();
        } 
        if(Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.1) != 0) {
            if(topLS.get()) {
                if(Utilities.deadband(controller.getY(Hand.kLeft), 0.1) <= 0) {
                    elevatorTalon.set(ControlMode.PercentOutput, Utilities.deadband(controller.getY(Hand.kLeft), 0.1));
                }
            } else if(bottomLS.get()) {
                if(Utilities.deadband(controller.getY(Hand.kLeft), 0.1) >= 0) {
                    elevatorTalon.set(ControlMode.PercentOutput, Utilities.deadband(controller.getY(Hand.kLeft), 0.1));
                }
            } else {
                elevatorTalon.set(ControlMode.PercentOutput, Utilities.deadband(controller.getY(Hand.kLeft), 0.1));
            }
        } 
        else if(controller.getBumper(Hand.kLeft) && targetPosition != levels[levels.length]) {
            targetPosition = levels[elevatorPosition.ordinal() + 1];
        } else if(controller.getBumper(Hand.kRight) && targetPosition != levels[0]) {
            targetPosition = levels[elevatorPosition.ordinal() - 1];
        } if(!completeManualOverride) {
            drive();
        }
        if(topLS.get() && lsActive && !completeManualOverride) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            lsActive = false;
        } else if(bottomLS.get() && lsActive && !completeManualOverride) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            lsActive = false;
        } if(!topLS.get() && !bottomLS.get()) {
            lsActive = true;
        }
    }
    //TODO: Implement Procedure for when encoderHealthy == false;
    //Likely Lidar PID Control
    public void commandElevator(ElevatorPosition position) {
        if(encoderHealthy && !atTarget())
            elevatorTalon.set(ControlMode.MotionMagic, position.getTargetEncValue());
    }

    public void commandElevator(int buttonPressed) {
        super.push(new ElevatorCmd(elevatorMap.get(buttonPressed).getElevatorPosition(), elevatorMap.get(buttonPressed).getTargetAzimuth()));
        if(elevatorMap.get(buttonPressed).getTargetIntakePosition() != null)
            Intake.getInstance().changeIntakeMode(elevatorMap.get(buttonPressed).getTargetIntakePosition());
    }

    public boolean atTarget() {
        return elevatorPosition == targetPosition && 
        encoderHealthy ? (getEncDelta(elevatorPosition) < Constants.ENC_THRESHOLD) && (getLidarDelta(elevatorPosition) < Constants.LIDAR_THRESHOLD)
            : (getLidarDelta(elevatorPosition) < Constants.LIDAR_THRESHOLD);
    }

    public int getEncDelta(ElevatorPosition position) {
        return Math.abs(encCount - position.getTargetEncValue());
    }
    public double getLidarDelta(ElevatorPosition position) {
        return Math.abs(lidarValue - position.getTargetLidarValue());
    }
    public IntakePosition getIntakePosition() {
        return intakeMode;
    }
    public ElevatorPosition getElevatorPosition() {
        return elevatorPosition;
    }

    public boolean onDemandTest() {
        double prevEncoderCount = elevatorTalon.getSelectedSensorPosition(1);
        boolean encoderHealthy = true;
        commandElevator(ElevatorPosition.FLOOR);
        Timer.delay(7);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            System.out.println("ELEVATOR ENCODER ERROR");
            System.out.println("   DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
            encoderHealthy = false;
        }
        commandElevator(ElevatorPosition.MID);
        Timer.delay(7);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            System.out.println("ELEVATOR ENCODER ERROR");
            System.out.println("    DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
            encoderHealthy = false;
        }
        commandElevator(ElevatorPosition.LOW);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            System.out.println("ELEVATOR ENCODER ERROR");
            System.out.println("    DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
            encoderHealthy = false;
        }
        Timer.delay(4);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) == 0) {
            System.out.println("ELEVATOR ENCODER ERROR");
            System.out.println("   DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
            encoderHealthy = false;
        }
        return encoderHealthy;
    }
}