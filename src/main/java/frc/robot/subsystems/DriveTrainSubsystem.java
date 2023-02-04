// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  public MotorController motorController00;
  public MotorController motorController01;
  public MotorController motorController02;
  public MotorController motorController03;

  private CANSparkMax sparkMax00;
  private CANSparkMax sparkMax01;
  private CANSparkMax sparkMax02;
  private CANSparkMax sparkMax03;
  
  private RelativeEncoder encoder00;
  private RelativeEncoder encoder01;
  private RelativeEncoder encoder02;
  private RelativeEncoder encoder03;

  private SparkMaxPIDController pidController00;
  private SparkMaxPIDController pidController01;
  private SparkMaxPIDController pidController02;
  private SparkMaxPIDController pidController03;

  /*
   * Auto-balancing taken from: https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_AutoBalance/src/org/usfirst/frc/team2465/robot/Robot.java
   */
  private AHRS navx_device;
  boolean autoBalanceXMode;
  boolean autoBalanceYMode; 
    
  /**
   * Runs the motors with arcade steering.
   */  
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;

  /*
   * Slew Rate Limiters to make Joystick inputs more gentle; 1/3 sec from 0 to 1
   */
  private final SlewRateLimiter m_speedLeftLimiter = new SlewRateLimiter(Constants.Joysticks.DriverLeft.rateLimit);
  private final SlewRateLimiter m_speedRightLimiter = new SlewRateLimiter(Constants.Joysticks.DriverRight.rateLimit);

  /**
   * Real Drive Train
   */
  public DifferentialDrive m_drive;

  /**
   * Simluation
   */
  private Trajectory m_trajectory;

  /**
   * PID
   */
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  /**
   * Autobalance
   */
  public double wheelDiameter = Constants.DriveTrain.wheelDiameter;
  public double platformWidth = Constants.DriveTrain.platformWidth;
  public double robotLength = Constants.DriveTrain.robotLength;
  public double minAngle = Constants.DriveTrain.minAngle;
  public double maxAngle = Constants.DriveTrain.maxAngle;
  public double minMovementSpeed = Constants.DriveTrain.minMovementSpeed;
  public double maxMovementSpeed = Constants.DriveTrain.maxMovementSpeed;
  
  public double rotationsPerInch = 1 / (wheelDiameter * Math.PI);
  public double distanceToEdge = platformWidth - ((platformWidth - robotLength) / 2);
  public double rotationsToBalance = distanceToEdge * rotationsPerInch;

  public double slope = (maxMovementSpeed - minMovementSpeed) / (maxAngle - minAngle);

  public double rotationsNeededLeft = 0.0;
  public double rotationsNeededRight = 0.0;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    /*********************************
     * GYROSCOPE
     *********************************/
    try {
      if (RobotBase.isReal()) {
        navx_device = new AHRS(SerialPort.Port.kUSB);
      }
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP", true);
    }

    /*********************************
     * MOTORS
     *********************************/
    sparkMax00 = new CANSparkMax(
      Constants.DriveTrain.canMotorControllerPort00, 
      MotorType.kBrushless
    );

    sparkMax01 = new CANSparkMax(
      Constants.DriveTrain.canMotorControllerPort01, 
      MotorType.kBrushless
    );

    sparkMax02 = new CANSparkMax(
      Constants.DriveTrain.canMotorControllerPort02, 
      MotorType.kBrushless
    );

    sparkMax02.setInverted(true);

    sparkMax03 = new CANSparkMax(
      Constants.DriveTrain.canMotorControllerPort03, 
      MotorType.kBrushless
    );

    sparkMax03.setInverted(true);

    sparkMax01.follow(sparkMax00);
    sparkMax03.follow(sparkMax02);

    encoder00 = sparkMax00.getEncoder();
    encoder02 = sparkMax02.getEncoder();

    pidController00 = sparkMax00.getPIDController();
    pidController02 = sparkMax02.getPIDController();

    pidController00.setP(Constants.DriveTrain.kP);
    pidController00.setI(Constants.DriveTrain.kI);
    pidController00.setD(Constants.DriveTrain.kD);
    pidController00.setIZone(Constants.DriveTrain.kIz);
    pidController00.setFF(Constants.DriveTrain.kFF);

    pidController01.setP(Constants.DriveTrain.kP);
    pidController01.setI(Constants.DriveTrain.kI);
    pidController01.setD(Constants.DriveTrain.kD);
    pidController01.setIZone(Constants.DriveTrain.kIz);
    pidController01.setFF(Constants.DriveTrain.kFF);

    // rightMotors.setInverted(true);

    /**
     * SIMULTATION
     */
    m_trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2, 2, new Rotation2d()),
      List.of(),
      new Pose2d(6, 4, new Rotation2d()),
      new TrajectoryConfig(2, 2)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * drive()
   * 
   * Custom drive functionality
   * 
   * @param joystickLeftSpeed
   * @param joystickRightSpeed
   */
  public void drive(double joystickLeftSpeed, double joystickRightSpeed) {
    double m_deadband = Constants.DriveTrain.kDefaultDeadband;
    double m_maxOutput = Constants.DriveTrain.kDefaultMaxOutput;

    // Used to set 0.0 if joysticks withing the deadband area
    double leftSpeed = MathUtil.applyDeadband(joystickLeftSpeed, m_deadband);
    double rightSpeed = MathUtil.applyDeadband(joystickRightSpeed, m_deadband);

    WheelSpeeds wheelSpeeds; 

    // Limit Max Speed
    leftSpeed = MathUtil.clamp(leftSpeed, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);

    // This might not be needed, but it's in the DifferentialDrive code
    wheelSpeeds = new WheelSpeeds(leftSpeed, rightSpeed);

    // Set the value on the leader motors.
    sparkMax00.set(wheelSpeeds.left * m_maxOutput);
    sparkMax02.set(wheelSpeeds.right * m_maxOutput);

    return;
  }
  
  public void autoBalanceInitialize() {
    rotationsNeededLeft = encoder00.getPosition() + rotationsToBalance;
    rotationsNeededRight = encoder02.getPosition() + rotationsToBalance;
  }

  /**
   * autoBalance()
   * 
   * Auto-balance robot.
   * 
   * This should be run either as a button hold or as a stop command.
   * 
   * Modified from: https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_AutoBalance/src/org/usfirst/frc/team2465/robot/Robot.java
   */
  public boolean autoBalance() {
    boolean status = true;

    double brakeAdjustment = Constants.DriveTrain.brakeAdjustment;

    // Y-axis rotation
    double pitchAngleDegrees = navx_device.getPitch();

    // X-axis rotation
    double rollAngleDegrees = navx_device.getRoll();

    rollAngleDegrees = MathUtil.applyDeadband(rollAngleDegrees, 0.05);

    if (rollAngleDegrees != 0.0) {
      double radiansPerAngle = (slope * rollAngleDegrees);

      double direction = 1.0;
      if (rollAngleDegrees < 0.0) {
        direction = -1.0;
      }

      radiansPerAngle = radiansPerAngle + (minMovementSpeed * direction);

      try {
        double encoder00Position = encoder00.getPosition();
        double encoder02Position = encoder02.getPosition();

        double leftSpeed;
        if (encoder00Position >= rotationsNeededLeft) {
          leftSpeed = radiansPerAngle - (brakeAdjustment * direction);
        } else {
          leftSpeed = radiansPerAngle;
        }

        double rightSpeed;
        if (encoder02Position >= rotationsNeededRight) {
          rightSpeed = radiansPerAngle - (brakeAdjustment * direction);
        } else {
          rightSpeed = radiansPerAngle;
        }

        drive(leftSpeed, rightSpeed);

      } catch(RuntimeException ex) {
        String err_string = "Drive system error:  " + ex.getMessage();
        DriverStation.reportError(err_string, true);
        status = false;
      }
    }
    
    Timer.delay(0.005);

    return status;
  }
}
