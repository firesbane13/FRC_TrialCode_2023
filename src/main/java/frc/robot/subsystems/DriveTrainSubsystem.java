// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

    /**
     * Real Drive Train
     */
    public DifferentialDrive m_drive;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    /*********************************
     * GYROSCOPE
     *********************************/
    try {
      if (RobotBase.isReal()) {
        navx_device = new AHRS();
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

    sparkMax03 = new CANSparkMax(
      Constants.DriveTrain.canMotorControllerPort03, 
      MotorType.kBrushless
    );

    motorController00 = sparkMax00;
    motorController01 = sparkMax01;
    motorController02 = sparkMax02;
    motorController03 = sparkMax03;

    leftMotors = new MotorControllerGroup(
      motorController00,
      motorController01
    );

    rightMotors = new MotorControllerGroup(
      motorController02,
      motorController03
    );

    m_drive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  }


  /**
   * tankDrive()
   * 
   * Functionality to implement tank drive.
   * 
   * @param leftSpeed
   * @param rightSpeed
   * @return
  */
  public boolean tankDrive(double leftSpeed, double rightSpeed) {
    boolean status = true;

    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);

    m_drive.tankDrive(leftSpeed, rightSpeed);

    return status;
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

    double xAxisRate = 0.0;
    double yAxisRate = 0.0;

    // Retrieve robot angle in 3D space
    double pitchAngleDegrees = navx_device.getPitch();
    double rollAngleDegrees = navx_device.getRoll();

    if ( !autoBalanceXMode && 
      (Math.abs(rollAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))
    ) {
      autoBalanceXMode = true;
    } else if ( autoBalanceXMode && 
      (Math.abs(rollAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))
    ) {
      autoBalanceXMode = false;
    }

    if ( !autoBalanceYMode && 
      (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))
    ) {
      autoBalanceYMode = true;
    } else if ( autoBalanceYMode && 
      (Math.abs(pitchAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))
    ) {
      autoBalanceYMode = false;
    }

    /* 
     * Control drive system automatically, 
     * driving in reverse direction of pitch/roll angle,
     * with a magnitude based upon the angle
     */
    if ( autoBalanceXMode ) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(rollAngleRadians) * -1;
    }

    if ( autoBalanceYMode ) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(pitchAngleRadians) * -1;
    }

    try {
      m_drive.arcadeDrive(yAxisRate, xAxisRate);
    } catch(RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
      status = false;
    }
    
    Timer.delay(0.005);

    return status;
  }
}
