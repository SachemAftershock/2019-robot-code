package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Limelight.CameraMode;

public class Robot extends TimedRobot {

  SWDrive driveBase;
  Compressor compressor;

  @Override
  public void robotInit() {
    driveBase = SWDrive.getInstance();

    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
    Limelight.setCameraMode(CameraMode.eVision);
    driveBase.zero();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    driveBase.zero(); //TODO: remove after testing
  }

  @Override
  public void teleopPeriodic() {
    driveBase.drive();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
