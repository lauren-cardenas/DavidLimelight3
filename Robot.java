/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand; 
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  private final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(Map.m_leftMotor1);
  private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(Map.m_leftMotor2);
  private final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(Map.m_rightMotor1);
  private final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(Map.m_rightMotor2);

  private final SpeedController m_leftMotors = new SpeedControllerGroup(m_leftMotor1, m_leftMotor2);
  private final SpeedController m_rightMotors = new SpeedControllerGroup(m_rightMotor1, m_rightMotor2);

  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final Encoder m_leftEncoder = new Encoder(Map.m_leftEnc1, Map.m_leftEnc2);
  private final Encoder m_rightEncoder = new Encoder(Map.m_rightEnc1, Map.m_rightEnc2);

  //CONTROLLER junk

  private final XboxController m_driverController = new XboxController(Map.DRIVER_CONTROLLER);
  private final XboxController m_operatorController = new XboxController(Map.OPERATOR_CONTROLLER);

  //GAME TIMER

  private final Timer m_timer = new Timer();

  //LIMELIGHT JUNK

  private boolean m_LimelightHasValidTarget = false; 
  private double m_LimelightDriveCommand = 0.0; 
  private double m_LimelightSteerCommand = 0.0; 
  private double tv, tx, ty, ta; 
  private double error, integralError, derivativeError, previousError; 

  //Constants to tune/set 

  private static final double kSteer = 0.12; // how hard to turn toward the target
  private static final double kProportional = 0.80; // proportional constant
  private static final double kIntegral = 0.00; // integral constant 
  private static final double kDerivative = 0.00; // derivative constant 
  private static final double kFeedForward = 0.05; // feedforward constant  
  private static final double kMaxDrive = 0.60; // simple speed limit so we dont drive too fast 
  private static final double kMaxTurn = 0.70;  // ^ turn limit 
  private static final double kTargetHeight = 92; // height of target above floor 
  private static final double kCameraHeight = 47.5; //height of camera above the floor
  private static final double kMountingAngle = 30; //angle that camera is mounted at 

  //Close Target Junk 

  private static final double [] kTargetAreas = { 6.0, 2.5, 0.85 }; 
  private static final double [] kTargetDistance = { 20, 40, 60 }; 
  private static final double [] distancesToTarget = new double[kTargetAreas.length]; 
  private static double closestTargetArea = 0;
  private static double closestTargetDistance = 0; 
  

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  
  @Override
  public void robotPeriodic() {
  }

 
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  
  @Override
  public void teleopPeriodic() {
    //GTA drive 
    final double triggerVal = (m_driverController.getTriggerAxis(Hand.kRight)
        - m_driverController.getTriggerAxis(Hand.kLeft)) * Map.DRIVING_SPEED;

    final double stick = (m_driverController.getX(Hand.kLeft)) * Map.TURNING_RATE;

    double left_command = (triggerVal + stick) * Map.DRIVING_SPEED;
    double right_command = (triggerVal - stick) * Map.DRIVING_SPEED;

    m_driveTrain.tankDrive(left_command, right_command);
    
    //Limelight Junk 

      updateLimelightTracking(); 

      double steer = m_driverController.getX(Hand.kRight) * kMaxTurn;
      double drive = m_driverController.getY(Hand.kLeft) * kMaxDrive; 

      if(m_driverController.getAButton()) {
        moveWithLimelight(getClosestTargetArea(kTargetAreas, ta)); //??
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      } else if (m_driverController.getBButton()) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        m_driveTrain.tankDrive(0,0);
      } else {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        SmartDashboard.putNumber("Closest Target (Area)", 0);
        SmartDashboard.putNumber("Closest Target (Distance)", 0); 
        m_driveTrain.tankDrive(drive, steer);
      }
  }

  }

  @Override
  public void testPeriodic() {
  }
  public void updateLimelightTracking() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); 
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); 
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0); 

    SmartDashboard.putNumber("Target Detected", tv); 
    SmartDashboard.putNumber("Horizontal Error", tx);
    SmartDashboard.putNumber("Vertical Error", ty); 
    SmartDashboard.putNumber("Target Area", ta);
    SmartDashboard.putNumber("Closest Target (Area)", closestTargetArea);
    SmartDashboard.putNumber("Closest Target(Distance", closestTargetDistance); 

    // using target area
    closestTargetArea = getClosestTargetArea(kTargetAreas, ta); 

    //using target distance 
    closestTargetDistance = getClosestTargetDistance(kTargetDistance, distancesToTarget(ty)); 

    if (tv < 1.0) { // if target is detected 
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0; 
      m_LimelightSteerCommand = 0.0; 
      return; 

    }

    m_LimelightHasValidTarget = true; 
 
  }
  //function that returns the distance to the target 
  public double distancesToTarget(double targetAngle) {

    return (kTargetHeight - kCameraHeight) / Math.tan(kMountingAngle + targetAngle); 
  }

  public void moveWithLimelight(double targetArea) {
    //use a proportional loop for steering 
    double steerError = tx * kSteer; 
    m_LimelightSteerCommand = steerError; 

    // try to drive fowardd until the target area reaches our desired area 
    error = targetArea - ta; 
    integralError += error * 0.02; 
    derivativeError = (error*kProportional) + (integralError*kIntegral) + (derivativeError*kDerivative) + kFeedForward; 

    //dont let the robot drive too fast into the goal 
    if(driveError > kMaxDrive) { 
      driveError = kMaxDrive; 
      
    }
  } 
}
