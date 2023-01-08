// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutobalanceCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;

  /** Creates a new AutobalanceCommand. */
  public AutobalanceCommand(DriveTrainSubsystem driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrainSubsystem = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrainSubsystem.autoBalance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
