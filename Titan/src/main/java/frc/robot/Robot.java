package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  XboxController primaryDriver;//, secondaryDriver;
  SWDrive drive;
  Compressor compressor;

  @Override
  public void robotInit() {
    primaryDriver = new XboxController(Constants.PRIMARY_DRIVER_PORT);
    //secondaryDriver = new XboxController(Constants.SECONDARY_DRIVER_PORT);
    drive = SWDrive.getInstance();
    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drive.drive(primaryDriver);

    if(primaryDriver.getAButton())
      drive.setHighGear();
    if(primaryDriver.getBButton())
      drive.setLowGear();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
