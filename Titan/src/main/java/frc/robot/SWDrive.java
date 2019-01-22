package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;

class SWDrive {
    private TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
    private DoubleSolenoid gearSolenoid;
    private boolean tankEnabled;
    private AHRS navx;

    private static SWDrive driveInstance = new SWDrive();
    private PIDController controller;
    private boolean setpointReached;

    private SWDrive() {
        leftMaster = new TalonSRX(Constants.LEFT_MASTER_PORT);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        leftMaster.setSelectedSensorPosition(0, 0, 0);

        rightMaster = new TalonSRX(Constants.RIGHT_MASTER_PORT);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);

        leftSlave = new TalonSRX(Constants.LEFT_SLAVE_PORT);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        leftSlave.follow(leftMaster);

        rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_PORT);
        rightSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.follow(rightMaster);

        gearSolenoid = new DoubleSolenoid(Constants.DRIVE_SOLENOID_FORWARD, Constants.DRIVE_SOLENOID_REVERSE);

        navx = new AHRS(Port.kMXP);
        controller = new PIDController();
        tankEnabled = false;
        setpointReached = true;
    }

    public static SWDrive getInstance() {
        return driveInstance;
    }

    public void drive(XboxController controller) {
        System.out.println(navx.getYaw());

        if(controller.getBackButtonReleased()) {
            tankEnabled = !tankEnabled;
        }

        if(tankEnabled) {
            tankDrive(controller);
        } else {
            arcadeDrive(controller);
        }

        if(controller.getXButtonReleased() && gearSolenoid.get() != Value.kForward) {
            gearSolenoid.set(Value.kForward);
        } else if(controller.getAButtonReleased() && gearSolenoid.get() != Value.kReverse) {
            gearSolenoid.set(Value.kReverse);
        }
    }

    private void tankDrive(XboxController controller) {
        double leftY = Utilities.deadband(controller.getY(Hand.kLeft), 0.1);
        double rightY = Utilities.deadband(controller.getY(Hand.kRight), 0.1);
        double leftTrigger = Utilities.deadband(controller.getTriggerAxis(Hand.kLeft), 0.1);
        double rightTrigger = Utilities.deadband(controller.getTriggerAxis(Hand.kRight), 0.1);

        double leftSpeed = leftY - leftTrigger + rightTrigger;
        double rightSpeed = rightY + leftTrigger - rightTrigger;
        double[] speeds = {leftSpeed, rightSpeed};
        speeds = Utilities.normalize(speeds);

        driveMotors(speeds[0], speeds[1]);
    }

    private void arcadeDrive(XboxController controller) {
        double leftY = Utilities.deadband(controller.getX(Hand.kRight), 0.1);
        double rightX = -Utilities.deadband(controller.getY(Hand.kLeft), 0.1);

        double leftSpeed = leftY  + rightX;
        double rightSpeed = leftY - rightX;
        double[] speeds = {leftSpeed, rightSpeed};
        speeds = Utilities.normalize(speeds);
 
        driveMotors(speeds[0], speeds[1]);
    }

    private void driveMotors(double leftSpeed, double rightSpeed) {
        leftMaster.set(ControlMode.PercentOutput, leftSpeed);
        rightMaster.set(ControlMode.PercentOutput, rightSpeed);
    } 

    public void autoDrive(double naturalUnits) {
        if(setpointReached)
            setpointReached = false;

        leftMaster.set(ControlMode.Position, naturalUnits);
        rightMaster.set(ControlMode.Position, naturalUnits);
    }

    public void autoRotate(double theta) {
        if(setpointReached) {
            setpointReached = false;
            controller.start(Constants.ROTATE_GAINS, theta, navx.getYaw());
        }

        double output = controller.update(navx.getYaw());
        driveMotors(output, output); //Normally driveMotors(output, -output); but right side is inverted
        setpointReached = Math.abs(controller.getError()) < Constants.ROTATE_EPSILON;
    }

    public boolean setpointReached() {
        System.out.println("Setpoint Reached: " + setpointReached);
        return setpointReached;
    }

    public void zero() {
        navx.zeroYaw();
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public void zeroYaw() {
        navx.zeroYaw();
    }

    class PIDController {

        private double kP, kI, kD, setpoint, integral, derivative, error, prevError;
        private Timer timer;
        public PIDController() {
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
            setpoint = 0.0;
            integral = 0.0;
            derivative = 0.0;
            error = 0.0;
            prevError = 0.0;

            timer = new Timer();
        }

        public double getError() {
            return error;
        }

        public double getRuntime() {
            return timer.get();
        }

        public double update(double current) {
            error = setpoint - current;
            integral += error;
            derivative = error - prevError;

            double output = kP * error + kI * integral + kD * derivative;

            prevError = error;
            System.out.println("PIDController::Update error: " + error + ", prevError: " + prevError + ", integral: " + integral + ", derivative: " + derivative + ", setpoint: " + setpoint + ", output: " + output );
            return Math.abs(output) > 1.0 ? output / output : output;
        }

        public void start(double[] gains, double setpoint, double current) {
            kP = gains[0];
            kI = gains[1];
            kD = gains[2];
            this.setpoint = setpoint;
            integral = 0.0;
            derivative = 0.0;
            error = 0.0;
            prevError = 0.0;
            timer.start();
            
        }
    }

}