package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {

  SWDrive driveBase;
  Elevator elevator;
  Intake intake;
  Climber climber;

  Compressor compressor;
  XboxController sDriver;

  @Override
  public void robotInit() {
    driveBase = SWDrive.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();
    climber = Climber.getInstance();

    sDriver = new XboxController(Constants.SECONDARY_DRIVER_PORT);

    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
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
        driveBase.onDemandTest();
        elevator.onDemandTest();
      intake.onDemandTest();
      Timer.delay(20);
      if(Utilities.deadband(sDriver.getTriggerAxis(Hand.kLeft), 0.1) > 0) {
        climber.onDemandTest();
      }
    }
  }
}
