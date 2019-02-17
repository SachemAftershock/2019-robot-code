package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class Elevator extends Mechanism {
    private static Elevator instance = new Elevator();
    private Intake intake;
    private IntakePosition intakeMode;
    private ElevatorPosition currentElevatorPosition, previousElevatorPosition, targetPosition;
    private TalonSRX elevatorTalon;
    private DigitalInput topLS, bottomLS; //TODO: Find out how the Phoenix Lib integrates the LS
    private Joystick buttonBox;
    private LIDAR lidar;
    private int[] buttonID;
    private frc.robot.PIDController cargoPID, hatchPID;
    private boolean completeManualOverride, encoderHealthy, hatchModeEnabled, topLSPressed, bottomLSPressed, lidarBias, firstTime, setpointReached;
    private int previousEncoderCount, idOfLastButtonPressed;
    private double lidarValue, manualSpeedInput;

    private Map<Integer, ElevatorInfo> elevatorMap;

    public static Elevator getInstance() {
        return instance;
    }
    
    private Elevator() {
        super();
        intake = Intake.getInstance();
        currentElevatorPosition = ElevatorPosition.FLOOR;
        targetPosition = ElevatorPosition.FLOOR;
        intakeMode = IntakePosition.CARGO;
        buttonBox = new Joystick(Constants.BUTTON_BOX_PORT);
        lidar = new LIDAR(new DigitalInput(Constants.ELEVATOR_LIDAR_PORT));
        elevatorTalon = new TalonSRX(Constants.ELEVATOR_TALON_PORT);
        topLS = new DigitalInput(Constants.TOP_LS_PORT);
        bottomLS = new DigitalInput(Constants.BOTTOM_LS_PORT);
        elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.setInverted(true);
        elevatorTalon.configMotionAcceleration(1000, 0);
        elevatorTalon.configMotionCruiseVelocity(5000, 0);
        elevatorTalon.configClosedloopRamp(0, 256);
        elevatorTalon.configOpenloopRamp(0, 256);

        elevatorTalon.config_kP(0, Constants.HATCH_ELEVATOR_GAINS[0], 0);
        elevatorTalon.config_kI(0, Constants.HATCH_ELEVATOR_GAINS[1], 0);
        elevatorTalon.config_kD(0, Constants.HATCH_ELEVATOR_GAINS[2], 0);
        elevatorTalon.config_kF(0, Constants.HATCH_ELEVATOR_GAINS[3], 0); 
        elevatorTalon.config_IntegralZone(0, 200, 0);
        elevatorTalon.configAllowableClosedloopError(0, Constants.ELEVATOR_ENC_THRESHOLD, 0);//git doesnt like to work

        elevatorTalon.config_kP(1, Constants.CARGO_ELEVATOR_GAINS[0], 0);
        elevatorTalon.config_kI(1, Constants.CARGO_ELEVATOR_GAINS[1], 0);
        elevatorTalon.config_kD(1, Constants.CARGO_ELEVATOR_GAINS[2], 0);
        elevatorTalon.config_kF(1, Constants.CARGO_ELEVATOR_GAINS[3], 0);
        elevatorTalon.config_IntegralZone(1, 200, 0);
        elevatorTalon.configAllowableClosedloopError(1, Constants.ELEVATOR_ENC_THRESHOLD, 0);

        cargoPID = new frc.robot.PIDController();
        hatchPID = new frc.robot.PIDController();


        previousEncoderCount = elevatorTalon.getSelectedSensorPosition(0);
        lidarValue = lidar.getDistanceCm();
        completeManualOverride = false;
        topLSPressed = false;
        bottomLSPressed = false;
        lidarBias = true;
        buttonID = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
        elevatorMap = new HashMap<Integer, ElevatorInfo>();
        elevatorMap.put(1, new ElevatorInfo(ElevatorPosition.FLOOR, null , -1));
        
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

    public void drive(XboxController controller) {
        previousEncoderCount = elevatorTalon.getSelectedSensorPosition(0);
        lidarValue = lidar.getDistanceCm();
        if(controller.getBackButtonPressed()) { //TODO: Add Driver Feedback
            completeManualOverride = !completeManualOverride;
            (new JoystickRumble(controller, 1)).start();
        } 
        if(controller.getStartButtonReleased()) { //TODO: Check if this is the right function
            super.flush();
            (new JoystickRumble(controller, 1)).start();
        } 

        manualSpeedInput = (Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.1) - Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.1));
        elevatorTalon.set(ControlMode.PercentOutput, manualSpeedInput);
        /*if(Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0) {
            elevatorTalon.set(ControlMode.PercentOutput, Utilities.deadband(controller.getY(Hand.kLeft), 0.2));
        }*/

        if(controller.getBumper(Hand.kRight) && targetPosition != ElevatorPosition.HIGH) {
            targetPosition = ElevatorPosition.values()[currentElevatorPosition.ordinal() + 1];
        } else if(controller.getBumper(Hand.kLeft) && targetPosition != ElevatorPosition.FLOOR) {
            targetPosition = ElevatorPosition.values()[currentElevatorPosition.ordinal() - 1];
        }   

        if(controller.getBumper(Hand.kLeft) && controller.getBumper(Hand.kRight)) {
            lidarBias = !lidarBias;
            (new JoystickRumble(controller, 1)).start();
        }

        drive();
        updateElevatorPosition();
    }  
    public void drive() {
        if(topLS.get() && !topLSPressed) {
            currentElevatorPosition = ElevatorPosition.HIGH;
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            topLSPressed = true;
        } else {
            topLSPressed = false;
        }
        if(bottomLS.get() && !bottomLSPressed) {
            currentElevatorPosition = ElevatorPosition.LOW;
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            topLSPressed = true;
        } else {
            topLSPressed = false;
        }

        for(int id : buttonID) {
            if(buttonBox.getRawButton(id)) {
                if(id != idOfLastButtonPressed) {
                    super.flush(); //Only the button box adds to the elevator queue, this allows the driver to overwrite his last input
                    updateElevatorQueue(id);
                    idOfLastButtonPressed = id;
                }
            }
        }  
        if(!lidarBias)
            System.out.println(elevatorTalon.getSelectedSensorPosition(0));
        else
            System.out.println(lidar.getDistanceCm());
        if(super.size() > 0 || (target != null && !atTarget())) {
            if(super.size() > 0 && (target == null || atTarget())) {
                target = super.pop();
            } 
            System.out.println(target);
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
                case TILTHATCH:
                    changeIntakeMode(IntakePosition.HATCH);
                    break;
                case TILTCARGO:
                    changeIntakeMode(IntakePosition.CARGO);
                    break;
                case TILTTOPROCKET:
                    changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
                    break;
                default:
                    DriverStation.reportError("something went wrong chief", false);
                    break;
            }
        } else if(super.size() == 0 && targetPosition != currentElevatorPosition) {
            commandElevator(targetPosition);
        }

        //This is toggled on and off
        //TODO: Change
        if(buttonBox.getRawButton(Constants.INTAKE_MODE_TOGGLE_BUTTON)) {
            hatchModeEnabled = true;
        } else {
            hatchModeEnabled = false;
        }

        if(intakeMode == IntakePosition.HATCH && hatchModeEnabled) {
            if(currentElevatorPosition == ElevatorPosition.HIGH) {
                intake.changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
            } else {
                intake.changeIntakeMode(IntakePosition.CARGO);
            }
        } else if (!hatchModeEnabled) {
            intake.changeIntakeMode(IntakePosition.HATCH);
        }

        if(currentElevatorPosition == ElevatorPosition.HIGH && intakeMode == IntakePosition.CARGO) {
            intake.changeIntakeMode(IntakePosition.TOP_ROCKET_TILT);
        }
    }

    //TODO: Implement Procedure for when encoderHealthy == false;
    //Likely Lidar PID Control
    public void commandElevator(ElevatorPosition targetPosition) {
        if(setpointReached) {
            setpointReached = false;
            if(intakeMode == IntakePosition.HATCH) {
                hatchPID.start(Constants.HATCH_ELEVATOR_GAINS);
            } else {
                cargoPID.start(Constants.CARGO_ELEVATOR_GAINS);
            }
        }
        if(lidarBias && !atTarget()) {
            double output;
            if(intakeMode == IntakePosition.HATCH) {
                output = hatchPID.update(lidar.getDistanceCm(), targetPosition.getTargetLidarValue());
                setpointReached = Math.abs(hatchPID.getError()) < Constants.LIDAR_THRESHOLD;
            } else {
                output = cargoPID.update(lidar.getDistanceCm(), targetPosition.getTargetLidarValue());
                setpointReached = Math.abs(cargoPID.getError()) < Constants.LIDAR_THRESHOLD;
            }
            System.out.println("---------------LIDAR ELEVATOR COMMAND: " + targetPosition);
            elevatorTalon.set(ControlMode.PercentOutput, output);
        } else if(encoderHealthy && !atTarget()) {
            System.out.println("---------------ENC ELEVATOR COMMANDED: " + targetPosition);
            elevatorTalon.set(ControlMode.MotionMagic, targetPosition.getTargetEncValue());
        }
    }

    private boolean atTarget() {
        if(target == null) {
            return false;
        }
        if(encoderHealthy) {
            return setpointReached && getEncDelta(currentElevatorPosition) <= Constants.ELEVATOR_ENC_THRESHOLD && getLidarDelta(currentElevatorPosition) <= Constants.LIDAR_THRESHOLD;
        } else {
            return setpointReached && getLidarDelta(currentElevatorPosition) <= Constants.LIDAR_THRESHOLD;
        }
    }
    
    private void updateElevatorPosition() {
        if(isEncoderValueInRange(ElevatorPosition.FLOOR)) {
            currentElevatorPosition = ElevatorPosition.FLOOR;
        } else if(isEncoderValueInRange(ElevatorPosition.LOW)) {
            currentElevatorPosition = ElevatorPosition.LOW;
        } else if(isEncoderValueInRange(ElevatorPosition.MID)) {
            currentElevatorPosition = ElevatorPosition.MID;
        } else if(isEncoderValueInRange(ElevatorPosition.HIGH)) {
            currentElevatorPosition = ElevatorPosition.HIGH;
        }
    }

    private boolean isEncoderValueInRange(ElevatorPosition desiredPosition) {
        return Math.abs(desiredPosition.getTargetEncValue() - elevatorTalon.getSelectedSensorPosition(0)) <= Constants.ELEVATOR_ENC_THRESHOLD;
    }

    private void updateElevatorQueue(int buttonPressed) {
        super.push(new ElevatorCmd(elevatorMap.get(buttonPressed).getElevatorPosition(), elevatorMap.get(buttonPressed).getTargetAzimuth()));
        if(elevatorMap.get(buttonPressed).getTargetIntakePosition() != null)
            Intake.getInstance().changeIntakeMode(elevatorMap.get(buttonPressed).getTargetIntakePosition());
    }

    public void updateTalonPIDProfile() {
        if(intakeMode == IntakePosition.CARGO) {
            elevatorTalon.selectProfileSlot(1, 0);
        } else if(intakeMode == IntakePosition.HATCH) {
            elevatorTalon.selectProfileSlot(0, 0);
        }
    }

    private int getEncDelta(ElevatorPosition position) {
        return Math.abs(elevatorTalon.getSelectedSensorPosition(0) - position.getTargetEncValue());
    }
    private double getLidarDelta(ElevatorPosition position) {
        return Math.abs(lidarValue - position.getTargetLidarValue());
    }
    private void changeIntakeMode(IntakePosition position) {
        intake.changeIntakeMode(position);
    }
    public IntakePosition getIntakePosition() {
        return intakeMode;
    }
    public void setIntakePosition(IntakePosition position) {
        intakeMode = position;
    }
    

    public void onDemandTest() {
        //TODO: Replace EncoderHealthy with LED Strip Action
        double prevEncoderCount = elevatorTalon.getSelectedSensorPosition(1);
        commandElevator(ElevatorPosition.FLOOR);
        Timer.delay(7);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            DriverStation.reportWarning("ELEVATOR ENCODER ERROR", false);
            System.out.println("   DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
        }
        commandElevator(ElevatorPosition.MID);
        Timer.delay(7);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            DriverStation.reportWarning("ELEVATOR ENCODER ERROR \n " + "    DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)), false);
            System.out.println("    DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
        }
        commandElevator(ElevatorPosition.LOW);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) < 10) {
            DriverStation.reportWarning("ELEVATOR ENCODER ERROR", false);
            System.out.println("    DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
        }
        Timer.delay(4);
        if(Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)) == 0) {
            DriverStation.reportWarning("ELEVATOR ENCODER ERROR", false);
            System.out.println("   DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));
        }
    }
}