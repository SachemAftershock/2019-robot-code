package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

class SWDrive {
    private TalonSRX /*frontLeftMotor,*/ frontRightMotor, /*backLeftMotor,*/ backRightMotor;
    private VictorSP frontLeftMotor, backLeftMotor;
    private DoubleSolenoid gearSolenoid;
    private boolean tankEnabled;

    private static SWDrive driveInstance = new SWDrive();

    private SWDrive() {
        frontLeftMotor = new VictorSP(Constants.FRONT_LEFT_MOTOR_PORT);
        frontRightMotor = new TalonSRX(Constants.FRONT_RIGHT_MOTOR_PORT);
        backLeftMotor = new VictorSP(Constants.BACK_LEFT_MOTOR_PORT);
        backRightMotor = new TalonSRX(Constants.BACK_RIGHT_MOTOR_PORT);

        gearSolenoid = new DoubleSolenoid(Constants.DRIVE_SOLENOID_FORWARD, Constants.DRIVE_SOLENOID_REVERSE);

        tankEnabled = false;
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

        //TODO: Need to test how get_ButtonReleased works,
        //      not sure if it runs how I think it does.
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
        frontLeftMotor.set(/*ControlMode.PercentOutput,*/ leftSpeed);
        backLeftMotor.set(/*ControlMode.PercentOutput,*/ leftSpeed);
        frontRightMotor.set(ControlMode.PercentOutput, rightSpeed);
        backRightMotor.set(ControlMode.PercentOutput, rightSpeed);
    } 

}