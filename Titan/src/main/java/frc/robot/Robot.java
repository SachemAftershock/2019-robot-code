package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Limelight.CameraMode;

public class Robot extends TimedRobot {
  SWDrive driveBase;
  //SimpleElevator simpleElevator;
  Superstructure superstructure;
  Compressor compressor;

  @Override 
  public void robotInit() {
    //Initialize Subsystems
    driveBase = SWDrive.getInstance();
    superstructure = Superstructure.getInstance();
    
    //simpleElevator = SimpleElevator.getInstance();


    PistonClimber.getInstance().pullInPistons();
    driveBase.zero();

    //Initialize Compressor
    compressor = new Compressor();
    compressor.start();
    compressor.setClosedLoopControl(true);

    //Initialize Cameras
    CameraServer.getInstance().startAutomaticCapture();
    Limelight.setCameraMode(CameraMode.eDriver);
  }

  @Override
  public void autonomousInit() {
    superstructure.init();
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

  /**
   * Common Routine shared between Autonomous and Teleop
   */
  public void commonPeriodic() {
    driveBase.drive();
    superstructure.drive();
  }

  @Override
  public void testInit() {
    superstructure.init();
  }

  @Override
  public void testPeriodic() {
    commonPeriodic();
  }
}
