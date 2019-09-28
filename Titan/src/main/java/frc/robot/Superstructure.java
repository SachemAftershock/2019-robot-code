package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Enums.AutoObjective;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class Superstructure extends Mechanism {
    private static Superstructure instance = new Superstructure();
    private XboxController controller;
    private GenericHID buttonBox;
    private Elevator elevator;
    private Intake intake;
    private int[] buttonID;
    private int lastButtonIDPressed;
    private boolean setpointReached, intakeArmsOpen, hatchGrabActive;
    private Map<Integer, ElevatorInfo> elevatorMap;

    private Superstructure() {
        controller = new XboxController(Constants.SECONDARY_DRIVER_PORT);
        buttonBox = new Joystick(Constants.BUTTON_BOX_PORT);

        elevator = Elevator.getInstance();
        intake = Intake.getInstance();

        buttonID = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
        setpointReached = true;
        intakeArmsOpen = false;
        hatchGrabActive = false;
        lastButtonIDPressed = -1;

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

    public void drive() {
        if(!setpointReached || super.size() > 0) {
            if(super.size() > 0 && (target == null || setpointReached)) {
                target = super.pop();
            }
            if(super.peek() == null) {
                super.pop();
            }

            switch(target.getObjective()) {
                case SHOOT_HATCH:
                    intake.shootHatch();
                    break;
                case TILT_WRIST_UP:
                    intake.tiltWristToPosition(IntakePosition.HATCH);
                    break;
                case TILT_WRIST_DOWN:
                    intake.tiltWristToPosition(IntakePosition.CARGO);
                    break;
                case TILT_TOP_ROCKET:
                    intake.tiltWristToPosition(IntakePosition.TOP_ROCKET_TILT);
                case MOVE_ELEVATOR:
                    elevator.moveElevator(target.getSetpoint());
                    break;
                default:
                    break;
            }
        }

        for(int id : buttonID) {
            if(buttonBox.getRawButton(id)) {
                if(id != lastButtonIDPressed) {
                    super.flush(); //TODO: Maybe this isn't the best idea since queue is shared?
                    super.push(new ElevatorCmd(elevatorMap.get(id).getElevatorPosition()));
                    lastButtonIDPressed = id;
                }
            }
        }

        if(controller.getBumper(Hand.kRight)) {
            intake.shootCargo();
        } else if(controller.getBumper(Hand.kLeft)) {
            intake.intakeCargo();
        } else {
            intake.stopIntake();
        }

        if(controller.getBButtonPressed()) {
            super.push(new IntakeCmd(AutoObjective.SHOOT_HATCH));
        } 

        if(controller.getStickButtonPressed(Hand.kRight)) {
            if(hatchGrabActive) {
                intake.releaseHatchGrab();
            } else {
                intake.activateHatchGrab();
            }
            hatchGrabActive = !hatchGrabActive;
        }

        if(controller.getStickButtonPressed(Hand.kLeft)) {
            intake.zeroWristEncoder();
            (new JoystickRumble(controller, 1)).start();
        }

        if(controller.getAButtonPressed()) {
            if(intakeArmsOpen) {
                intake.closeArms();
            } else {
                intake.openArms();
            }
            intakeArmsOpen = !intakeArmsOpen;
        }

        if(Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0) {
            elevator.driveElevator(-Utilities.deadband(controller.getY(Hand.kLeft), 0.2));
            intake.setWristSpeed(Utilities.deadband(controller.getY(Hand.kRight), 0.2) * .65);
        }

    
        if(controller.getYButtonPressed()) {
            if(Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.5) > 0) {
                intake.tiltWristUp(true);
            } else {
                intake.tiltWristUp(false);
            }
        } else if(controller.getXButtonPressed()) {
            if(Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.5) > 0) {
                intake.tiltWristDown(true);
            } else {
                intake.tiltWristDown(false);
            }
        } else {
            intake.stopWristTilt();
        }
        
        //TODO: Test below
        if(controller.getPOV() == 0) {
            super.push(new IntakeCmd(AutoObjective.TILT_WRIST_UP));
        } else if(controller.getPOV() == 180) {
            super.push(new IntakeCmd(AutoObjective.TILT_WRIST_DOWN));
        } else if(controller.getPOV() == 90) {
            super.push(new IntakeCmd(AutoObjective.TILT_TOP_ROCKET));
        }
    }

    public void init() {
        super.flush();
        intake.openArms();
        intake.activateHatchGrab();
        intake.retractHatchShooter();
        intake.zeroWristEncoder();
        intakeArmsOpen = true;
        hatchGrabActive = true;
        setpointReached = true;
    }

    public void setSetpointReached(boolean value) {
        setpointReached = value;
    }
    
    public boolean getSetpointReached() {
        return setpointReached;
    }

    public static Superstructure getInstance() {
        return instance;
    }
}
