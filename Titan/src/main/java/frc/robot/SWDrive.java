package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

class SWDrive {
    private TalonSRX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DoubleSolenoid gearSolenoid;
    private boolean isHighGear;

    private static SWDrive driveInstance = new SWDrive();
    private boolean tankEnabled;

    private SWDrive() {
        frontLeftMotor = new TalonSRX(Constants.FRONT_LEFT_MOTOR_PORT);
        frontRightMotor = new TalonSRX(Constants.FRONT_RIGHT_MOTOR_PORT);
        backLeftMotor = new TalonSRX(Constants.BACK_LEFT_MOTOR_PORT);
        backRightMotor = new TalonSRX(Constants.BACK_RIGHT_MOTOR_PORT);
    }

    public static SWDrive getInstance() {
        return driveInstance;
    }

    public void drive(XboxController controller) {
        if(controller.getBackButtonReleased()) {
            tankEnabled = !tankEnabled;
        }

        if(tankEnabled) {
            tankDrive(controller);
        } else {
            arcadeDrive(controller);
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
        double leftY = Utilities.deadband(controller.getY(Hand.kLeft), 0.1);
        double rightX = Utilities.deadband(controller.getX(Hand.kRight), 0.1);

        double leftSpeed = leftY  + rightX;
        double rightSpeed = leftY - rightX;
        double[] speeds = {leftSpeed, rightSpeed};
        speeds = Utilities.normalize(speeds);

        driveMotors(speeds[0], speeds[1]);
    }

    private void driveMotors(double leftSpeed, double rightSpeed) {
        frontLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
        backLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
        frontRightMotor.set(ControlMode.PercentOutput, rightSpeed);
        backRightMotor.set(ControlMode.PercentOutput, rightSpeed);
    } 

    public void setHighGear() {
        if(!isHighGear) {
            gearSolenoid.set(Value.kForward);
            isHighGear = true;
        }
    }
    
    public void setLowGear() {
        if(isHighGear) {
            gearSolenoid.set(Value.kReverse);
            isHighGear = false;
        }   
    }
}