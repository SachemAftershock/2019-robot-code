package frc.robot;

//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  SWDrive driveBase;
  Elevator elevator;
  //SimpleElevator sEle;
  //IntakeTilt tilt;
  Intake intake;
  //PistonClimber pistonClimber; //NOTE: Piston Climber is not used in this file; used in SWDrive
  //Climber climber;
  //private static NetworkTableInstance table = null;

  Compressor compressor;
  XboxController sDriver;

  @Override 
  public void robotInit() {
    driveBase = SWDrive.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();
    //tilt = IntakeTilt.getInstance();
    //sEle = SimpleElevator.getInstance();
    //pistonClimber = PistonClimber.getInstance();
    //climber = Climber.getInstance();

    sDriver = new XboxController(1);

    
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
    driveBase.zero();
  }

  @Override
  public void teleopPeriodic() {
    commonPeriodic();
  }

  public void commonPeriodic() {
    
    driveBase.drive();
    //sEle.drive(sDriver);
    elevator.drive(sDriver);
    intake.drive(sDriver);
    //analyzeVisionData();
    //tilt.drive(sDriver);
  }

 /* public void analyzeVisionData() {
    if (table == null) {
			table = NetworkTableInstance.getDefault();
		}
    double[] tapeCentroids = new double[5];
    tapeCentroids =  table.getTable("floorTapeData").getEntry("floorTapeCentroids").getDoubleArray(tapeCentroids);
    for(double x : tapeCentroids) {
      if(x != -1) {

      }
    }
  }*/

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    commonPeriodic();
  }
}
