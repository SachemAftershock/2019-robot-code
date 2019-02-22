package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Enums.AutoObjective;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class Elevator extends Mechanism {
    private static Elevator instance = new Elevator();
    private IntakePosition intakeMode;
    private ElevatorPosition currentElevatorPosition;//, targetPosition;
    private TalonSRX elevatorTalon;
    private DigitalInput topLS, bottomLS; //TODO: Find out how the Phoenix Lib integrates the LS
    private GenericHID buttonBox;
    private LIDAR lidar;
    private int[] buttonID;
    private frc.robot.PIDController cargoPID, hatchPID;
    private boolean completeManualOverride, hatchModeEnabled, topLSPressed, bottomLSPressed, setpointReached, firstRun, lidarHealthy;
    private int idOfLastButtonPressed;
    private boolean hatchModePrev = false;

    private Map<Integer, ElevatorInfo> elevatorMap;

    public static Elevator getInstance() {
        return instance;
    }
    
    private Elevator() {
        super();
        currentElevatorPosition = ElevatorPosition.FLOOR;
        //targetPosition = ElevatorPosition.FLOOR;
        intakeMode = IntakePosition.HATCH;
        buttonBox = new Joystick(Constants.BUTTON_BOX_PORT);
        lidar = new LIDAR(new DigitalInput(Constants.ELEVATOR_LIDAR_PORT));
        elevatorTalon = new TalonSRX(Constants.ELEVATOR_TALON_PORT);
        topLS = new DigitalInput(Constants.TOP_LS_PORT);
        bottomLS = new DigitalInput(Constants.BOTTOM_LS_PORT);
        elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.setInverted(false);
        elevatorTalon.configMotionAcceleration(1000, 0);
        elevatorTalon.configMotionCruiseVelocity(5000, 0);
        elevatorTalon.configClosedloopRamp(0, 256);
        elevatorTalon.configOpenloopRamp(0, 256);

        elevatorTalon.config_kP(0, Constants.HATCH_ELEVATOR_GAINS[0], 0);
        elevatorTalon.config_kI(0, Constants.HATCH_ELEVATOR_GAINS[1], 0);
        elevatorTalon.config_kD(0, Constants.HATCH_ELEVATOR_GAINS[2], 0);
        elevatorTalon.config_IntegralZone(0, 200, 0);
        elevatorTalon.configAllowableClosedloopError(0, Constants.ELEVATOR_ENC_THRESHOLD, 0);

        elevatorTalon.config_kP(1, Constants.CARGO_ELEVATOR_GAINS[0], 0);
        elevatorTalon.config_kI(1, Constants.CARGO_ELEVATOR_GAINS[1], 0);
        elevatorTalon.config_kD(1, Constants.CARGO_ELEVATOR_GAINS[2], 0);
        elevatorTalon.config_IntegralZone(1, 200, 0);
        elevatorTalon.configAllowableClosedloopError(1, Constants.ELEVATOR_ENC_THRESHOLD, 0);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);

        cargoPID = new frc.robot.PIDController();
        hatchPID = new frc.robot.PIDController();

        idOfLastButtonPressed = -1;
        completeManualOverride = false;
        topLSPressed = false;
        bottomLSPressed = false;
        lidarHealthy = true;
        firstRun = true;
        setpointReached = true;
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
        elevatorTalon.setSelectedSensorPosition(0);
        super.flush();
    }

    public void drive(XboxController controller) {
        if(firstRun) {
            super.flush();
            firstRun = false;
        }

        if(controller.getBackButtonPressed()) {
            completeManualOverride = !completeManualOverride;
            (new JoystickRumble(controller, 1)).start();
        } 
        if(controller.getStartButtonReleased()) {
            super.flush();
            (new JoystickRumble(controller, 1)).start();
        } 
        if(controller.getStickButton(Hand.kRight)) {
            elevatorTalon.setSelectedSensorPosition(0);
        }

        if(Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0) {
            elevatorTalon.set(ControlMode.PercentOutput, -Utilities.deadband(controller.getY(Hand.kLeft), 0.1));
        } else if(Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.5) != 0) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.1); //TODO: Tweak this value to nudge it up maybe?
        } else if(setpointReached && (Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0)) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
        }
        
        /*if(controller.getBumper(Hand.kRight) && currentElevatorPosition != ElevatorPosition.HIGH) {
            //targetPosition = ElevatorPosition.values()[currentElevatorPosition.ordinal() + 1];
            super.push(new ElevatorCmd(ElevatorPosition.values()[currentElevatorPosition.ordinal() + 1], -1));
        } else if(controller.getBumper(Hand.kLeft) && currentElevatorPosition != ElevatorPosition.FLOOR) {
            //targetPosition = ElevatorPosition.values()[currentElevatorPosition.ordinal() - 1];
            super.push(new ElevatorCmd(ElevatorPosition.values()[currentElevatorPosition.ordinal() - 1], -1));
        }  
        System.out.println("-------TARGET BUMPER:" + targetPosition);*/

        if(!(Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) > 0)) {
            drive();
        }
        checkLIDARHealth();
        updateElevatorPosition();
    }  
    public void drive() {
        /*if(!topLS.get() && !topLSPressed) { //TODO: Reimplement & Test LS Logic
            currentElevatorPosition = ElevatorPosition.HIGH;
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            topLSPressed = true;
        } else if (topLS.get() && topLSPressed) {
            topLSPressed = false;
        }
        if(!bottomLS.get() && !bottomLSPressed) {
            currentElevatorPosition = ElevatorPosition.FLOOR;
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            topLSPressed = true;
        } else if (bottomLS.get() && bottomLSPressed) {
            topLSPressed = false;
        }*/

        for(int id : buttonID) {
            if(buttonBox.getRawButton(id)) {
                if(id != idOfLastButtonPressed) {
                    super.flush(); //Only the button box adds to the elevator queue, this allows the driver to overwrite his last input
                    updateElevatorQueue(id);
                    idOfLastButtonPressed = id;
                }
            }
        }  

        if(super.size() > 0 || (target != null && !setpointReached)) {
            if(super.size() > 0 && (target == null || setpointReached)) {
                target = super.pop();
            } 
            if(super.peek() == null) {
                super.pop();
            }
            switch(target.getObjective()) {
                case ELEVATORFLOOR:
                    commandElevator(ElevatorPosition.FLOOR);
                    break;
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
                System.out.println("Unrecognized Elevator Command");
                    break;
            }
        } 

        if(buttonBox.getRawButton(Constants.INTAKE_MODE_TOGGLE_BUTTON)) {
            hatchModePrev= false | hatchModeEnabled;
            hatchModeEnabled = true;
        } else {
            hatchModePrev = false | !hatchModeEnabled;
            hatchModeEnabled = false;
        }

        if(intakeMode == IntakePosition.HATCH && hatchModeEnabled && intakeMode != IntakePosition.CARGO && !hatchModePrev) {
            if(currentElevatorPosition == ElevatorPosition.HIGH) {
                intakeMode = IntakePosition.CARGO;
            } else {
                hatchModePrev = true;
                intakeMode = IntakePosition.CARGO;
                
            }
        } else if (!hatchModeEnabled && intakeMode != IntakePosition.HATCH && !hatchModePrev) {
            intakeMode = IntakePosition.HATCH;
            hatchModePrev = true;
        }
    }

    public void commandElevator(ElevatorPosition targetPosition) {
        if(setpointReached) {
            if(intakeMode == IntakePosition.HATCH)
                hatchPID.start(Constants.HATCH_ELEVATOR_GAINS);
            else
                cargoPID.start(Constants.CARGO_ELEVATOR_GAINS);
            setpointReached = false;
        }

        if(!setpointReached && lidarHealthy) {
            double output;
            if(intakeMode == IntakePosition.HATCH) {
                output = hatchPID.update(lidar.getDistanceCm(), targetPosition.getTargetLidarValue());
                setpointReached = Math.abs(hatchPID.getError()) < Constants.LIDAR_THRESHOLD;
            } else {
                output = cargoPID.update(lidar.getDistanceCm(), targetPosition.getTargetLidarValue());
                setpointReached = Math.abs(cargoPID.getError()) < Constants.LIDAR_THRESHOLD;
            }
            elevatorTalon.set(ControlMode.PercentOutput, output);
        }
        if(setpointReached) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            //System.out.println("TARGET REACHED: " + targetPosition + " INTAKE: " + intakeMode);
            return;
        }
    }

    private void updateElevatorPosition() {
        if(lidarHealthy) {
            if(isLidarValueInRange(ElevatorPosition.FLOOR)) {
                currentElevatorPosition = ElevatorPosition.FLOOR;
            } else if(isLidarValueInRange(ElevatorPosition.LOW)) {
                currentElevatorPosition = ElevatorPosition.LOW;
            } else if(isLidarValueInRange(ElevatorPosition.MID)) {
                currentElevatorPosition = ElevatorPosition.MID;
            } else if(isLidarValueInRange(ElevatorPosition.HIGH)) {
                currentElevatorPosition = ElevatorPosition.HIGH;
            }
        }
    }
    private boolean isLidarValueInRange(ElevatorPosition desiredPosition) {
        return Math.abs(desiredPosition.getTargetLidarValue() - lidar.getDistanceCm()) <= Constants.LIDAR_THRESHOLD;
    }

    private void updateElevatorQueue(int buttonPressed) {
        if(elevatorMap.get(buttonPressed).getTargetIntakePosition() != null && elevatorMap.get(buttonPressed).getTargetIntakePosition() != intakeMode) {
            intakeMode = elevatorMap.get(buttonPressed).getTargetIntakePosition();
        }
        super.push(new ElevatorCmd(elevatorMap.get(buttonPressed).getElevatorPosition(), elevatorMap.get(buttonPressed).getTargetAzimuth()));
    }

    public void updateTalonPIDProfile() {
        if(intakeMode == IntakePosition.CARGO) {
            elevatorTalon.selectProfileSlot(1, 0);
        } else if(intakeMode == IntakePosition.HATCH) {
            elevatorTalon.selectProfileSlot(0, 0);
        }
    }

    private void checkLIDARHealth() {
        if(lidar.getDistanceCm() > 200 && lidarHealthy) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
            lidarHealthy = false;
        } else {
            lidarHealthy = true;
        }
    }

    public IntakePosition getIntakePosition() {
        return intakeMode;
    }
    public void setIntakePosition(IntakePosition position) {
        intakeMode = position;
    }
    

    //public void onDemandTest() {
        //TODO: Fix this
        /*double prevEncoderCount = elevatorTalon.getSelectedSensorPosition(1);
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
            System.out.println("   DIFF:" + Math.abs(prevEncoderCount - elevatorTalon.getSelectedSensorPosition(1)));*/
        //}
   // }
}