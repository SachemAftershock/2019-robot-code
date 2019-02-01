package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Enums.*;
import frc.robot.Auton.*;

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
    auto = Autonomous.getInstance();
    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);

    autoEnabled = true;
    drive.zero();
  }

  @Override
  public void autonomousInit() {
    drive.zero();

    auto.queueObjective(new AutonRotate(Rotation.CLOCKWISE, 90.0));
    auto.queueObjective(new AutonDrive(Distance.FEET, 6.0));
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
    drive.zero();//TODO: Remove this after testing
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
