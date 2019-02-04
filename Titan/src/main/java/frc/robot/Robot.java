package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  SWDrive driveBase;
  Elevator elevator;
  Intake intake;

  Compressor compressor;
  XboxController sDriver;

  @Override
  public void robotInit() {
    driveBase = SWDrive.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();

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
  }

  @Override
  public void teleopInit() {
 
  }

  @Override
  public void teleopPeriodic() {
    driveBase.drive();
    elevator.drive(sDriver);
    intake.drive(sDriver);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
