package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  SWDrive driveBase;
  Elevator elevator;
  Intake intake;
  //PistonClimber pistonClimber; //NOTE: Piston Climber is not used in this file; used in SWDrive
  //Climber climber;

  Compressor compressor;
  XboxController sDriver;

  @Override 
  public void robotInit() {
    driveBase = SWDrive.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();
    //pistonClimber = PistonClimber.getInstance();
    //climber = Climber.getInstance();

    sDriver = new XboxController(Constants.SECONDARY_DRIVER_PORT);

    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
    //Limelight.setCameraMode(CameraMode.eVision);
    driveBase.zero();
  }
 
  @Override
  public void autonomousPeriodic() {
    commonPeriodic();
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    commonPeriodic();
  }

  public void commonPeriodic() {
    driveBase.drive();
    elevator.drive(sDriver);
    intake.drive(sDriver);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    commonPeriodic();
    if(sDriver.getStartButton() && sDriver.getBackButton()) {
        //TODO: Probably Turn on LEDs in Error Mode
        //TODO: Use Dashboard or something to Process BIT
    }
  }
}
