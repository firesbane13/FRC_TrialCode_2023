// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private PWMSparkMax m_leftLeader = new PWMSparkMax(Constants.DriveTrain.controller01);
  private PWMSparkMax m_leftFollower = new PWMSparkMax(Constants.DriveTrain.controller02);
  private PWMSparkMax m_rightLeader = new PWMSparkMax(Constants.DriveTrain.controller03);
  private PWMSparkMax m_rightFollower = new PWMSparkMax(Constants.DriveTrain.controller04);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(
    m_leftLeader, 
    m_leftFollower
  );

  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(
    m_rightLeader, 
    m_rightFollower
  );

  private final Encoder m_leftEncoder = new Encoder(
    Constants.DriveTrain.encoderChannel00, 
    Constants.DriveTrain.encoderChannel01  
  );

  private final Encoder m_rightEncoder = new Encoder(
    Constants.DriveTrain.encoderChannel02, 
    Constants.DriveTrain.encoderChannel03
  );

  private final PIDController m_leftPIDController = new PIDController(
    8.5, 
    0, 
    0
  );

  private final PIDController m_rightPIDController = new PIDController(
    8.5, 
    0, 
    0
  );

  private final AnalogGyro m_gyro = new AnalogGyro(
    Constants.DriveTrain.gyro00
  );

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
