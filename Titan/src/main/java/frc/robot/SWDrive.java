package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.XboxController; 

class SWDrive extends Mechanism {

    private TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
    private DoubleSolenoid gearSolenoid;
    private XboxController controller;
    private boolean tankEnabled, setpointReached, rotateSet, cubeSearch;
    private AHRS navx;
    private double leftTarget, rightTarget;
    private static SWDrive driveInstance = new SWDrive();
    private PIDController pid;

    private SWDrive() {
        super();

        leftMaster = new TalonSRX(Constants.LEFT_MASTER_PORT);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        leftMaster.config_kP(0, Constants.LINEAR_GAINS[0], 0);
        leftMaster.config_kI(0, Constants.LINEAR_GAINS[1], 0);
        leftMaster.config_kD(0, Constants.LINEAR_GAINS[2], 0);
        leftMaster.configMotionAcceleration(1000, 0);
		leftMaster.configMotionCruiseVelocity(5000, 0);
		leftMaster.config_IntegralZone(0, 200, 0);
		leftMaster.configClosedloopRamp(0, 256);
		leftMaster.configOpenloopRamp(0, 256);
		leftMaster.configAllowableClosedloopError(0, Constants.LINEAR_EPSILON, 0);

        rightMaster = new TalonSRX(Constants.RIGHT_MASTER_PORT);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setInverted(true);
        rightMaster.config_kP(0, Constants.LINEAR_GAINS[0], 0);
        rightMaster.config_kI(0, Constants.LINEAR_GAINS[1], 0);
        rightMaster.config_kD(0, Constants.LINEAR_GAINS[2], 0);
        rightMaster.configMotionAcceleration(1000, 0);
		rightMaster.configMotionCruiseVelocity(5000, 0);
		rightMaster.config_IntegralZone(0, 200, 0);
		rightMaster.configClosedloopRamp(0, 256);
		rightMaster.configOpenloopRamp(0, 256);
		rightMaster.configAllowableClosedloopError(0, Constants.LINEAR_EPSILON, 0);

        leftSlave = new TalonSRX(Constants.LEFT_SLAVE_PORT);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        leftSlave.follow(leftMaster);

        rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_PORT);
        rightSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.setInverted(true);
        rightSlave.follow(rightMaster);

        gearSolenoid = new DoubleSolenoid(Constants.DRIVE_SOLENOID_FORWARD, Constants.DRIVE_SOLENOID_REVERSE);

        navx = new AHRS(Port.kMXP);
        controller = new XboxController(Constants.PRIMARY_DRIVER_PORT);
        pid = new PIDController();

        tankEnabled = false;
        setpointReached = true;
        rotateSet = false;
        cubeSearch = false;
        leftTarget = 0;
        rightTarget = 0;
    }

    public static SWDrive getInstance() {
        return driveInstance;
    }

    public void drive() {
        if(controller.getBackButtonReleased()) {
            setpointReached = true;
            super.flush();
        }

        if(!setpointReached || super.size() > 0) {
            if(target == null || setpointReached) {
                target = super.pop();
            }

            switch(target.getObjective()) {
                case DRIVELINEAR:
                    autoDrive(target.getSetpoint());
                    break;
                case DRIVEROTATE:
                    autoRotate(target.getSetpoint());
                    break;
                case CUBESEARCH:
                    rotateCube(target.getSetpoint());
                default:
                    break;
            }
        } else if(tankEnabled) {
            tankDrive();
        } else {
            arcadeDrive();
        }
       
        if(controller.getPOV() >= 0 && !rotateSet) {
            //TODO: bug below, make it get angle right before it sets motors
            super.push(new RotateCmd(controller.getPOV() + navx.getYaw()));
        }

        if(controller.getBumper(Hand.kLeft) && !cubeSearch) {
            super.push(new CubeSearchCmd(-1.0)); //TODO: Make enum for (counter)clockwise
        } else if(controller.getBumper(Hand.kRight) && !cubeSearch) {
            super.push(new CubeSearchCmd(1.0));
        }

        if(controller.getXButtonReleased() && gearSolenoid.get() != Value.kForward) {
            gearSolenoid.set(Value.kForward);
        } else if(controller.getAButtonReleased() && gearSolenoid.get() != Value.kReverse) {
            gearSolenoid.set(Value.kReverse);
        }


        if(controller.getStartButtonReleased()) {
            tankEnabled = !tankEnabled;
        }
        
        rotateSet = controller.getPOV() >= 0;
        cubeSearch = controller.getBumper(Hand.kLeft) || controller.getBumper(Hand.kRight);
    }

    public void autoDrive(double setpoint) {
        if(setpointReached) {
            leftTarget = leftMaster.getSelectedSensorPosition() + setpoint;
            rightTarget = rightMaster.getSelectedSensorPosition() + setpoint;
            setpointReached = false;
        }

        leftMaster.set(ControlMode.Position, leftTarget);
        rightMaster.set(ControlMode.Position, rightTarget);
        
        setpointReached = (Math.abs(leftMaster.getSelectedSensorPosition() - leftTarget) < Constants.LINEAR_EPSILON && Math.abs(rightMaster.getSelectedSensorPosition() - rightTarget) < Constants.LINEAR_EPSILON);
    }

    public void rotateCube(double direction) {
        if(setpointReached) {
            setpointReached = false;
            pid.start(Constants.ROTATE_GAINS);
        }

        if(Limelight.isTarget()) {
            double normAngle = Utilities.normalizeAngle(Limelight.getTx());
            double output = pid.updateRotation(navx.getYaw(), normAngle);

            driveMotors(output, -output);
        } else {
            double speed = direction * Constants.CUBE_SEARCH_ROTATE_SPEED;
            driveMotors(speed, -speed);
        }

        setpointReached = Limelight.isTarget() && Math.abs(Limelight.getTx()) < Constants.CUBE_SEARCH_EPSILON;
    }

    public void autoRotate(double theta) {
        if(setpointReached) {
            setpointReached = false;
            pid.start(Constants.ROTATE_GAINS);
        }
        double output = pid.updateRotation(navx.getYaw(), Utilities.normalizeAngle(theta));

        driveMotors(output, -output);

        setpointReached = Math.abs(pid.getError()) < Constants.ROTATE_EPSILON;
    }

    public void zero() {
        navx.zeroYaw();
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    private void tankDrive() {
        
        double leftY = Utilities.deadband(controller.getY(Hand.kLeft), 0.1);
        double rightY = Utilities.deadband(controller.getY(Hand.kRight), 0.1);
        double leftTrigger = Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.1);
        double rightTrigger = Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.1);

        double leftSpeed = leftY - leftTrigger + rightTrigger;
        double rightSpeed = rightY + leftTrigger - rightTrigger;
        double[] speeds = {-leftSpeed, -rightSpeed};

        driveMotors(Utilities.normalize(speeds));
    }

    private void arcadeDrive() {
        double leftY = -Utilities.deadband(controller.getY(Hand.kLeft), 0.2);
        double rightX = Utilities.deadband(controller.getX(Hand.kRight), 0.1);

        double leftSpeed = leftY + rightX;
        double rightSpeed = leftY - rightX;   
        double[] speeds = {leftSpeed, rightSpeed};
        
        driveMotors(Utilities.normalize(speeds));
    }

    private void driveMotors(double[] speeds) {
        driveMotors(speeds[0], speeds[1]);
    }

    private void driveMotors(double leftSpeed, double rightSpeed) {
        leftMaster.set(ControlMode.PercentOutput, leftSpeed);
        rightMaster.set(ControlMode.PercentOutput, rightSpeed);
    } 
}