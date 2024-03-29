// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Elevator;

public class ElevateUp extends Command {

  /** Creates a new ElevateUp. */
  private final Elevator elevator;
  public ElevateUp(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator); //here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.set(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
