package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {

  //SWDrive driveBase;
  CANSparkMax spark;
  XboxController controller;
  //Compressor compressor;

  @Override 
  public void robotInit() {
    //driveBase = SWDrive.getInstance();
    spark = new CANSparkMax(9, MotorType.kBrushless);
    controller = new XboxController(0);
    //compressor = new Compressor();
    //compressor.start();
    //compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
    //Limelight.setCameraMode(CameraMode.eVision);
    //driveBase.zero();
  }
 
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    //driveBase.zero(); //TODO: remove after testing
  }

  @Override
  public void teleopPeriodic() {
    spark.set(Utilities.deadband(controller.getY(Hand.kLeft), 0.1));
    System.out.println(spark.getEncoder().getPosition());
    //driveBase.drive();

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
