package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  XboxController primaryDriver, secondaryDriver;
  SWDrive drive;
  Autonomous auto;
  Compressor compressor;

  boolean autoEnabled;

  @Override
  public void robotInit() {
    primaryDriver = new XboxController(Constants.PRIMARY_DRIVER_PORT);
    secondaryDriver = new XboxController(Constants.SECONDARY_DRIVER_PORT);
    drive = SWDrive.getInstance();
    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);

    autoEnabled = true;
    drive.zero();
  }

  @Override
  public void autonomousInit() {
    auto.queueAutoRotate(Rotation.CLOCKWISE, 45);
  }

  @Override
  public void autonomousPeriodic() {
      if(autoEnabled) {
        auto.drive();
      } else {
        drive.drive(primaryDriver);
      }

      if(primaryDriver.getAButton()) { //TODO: change toggle button
        autoEnabled = false;
      }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drive.drive(primaryDriver);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
