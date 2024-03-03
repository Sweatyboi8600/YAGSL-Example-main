// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Shooter;

public class Shoot extends Command {
  private final Shooter Shooter;
  private  DoubleSupplier Foreward, Throttle;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, DoubleSupplier foreward, DoubleSupplier throttle) {
    this.Shooter = shooter;
    this.Foreward = foreward;
    this.Throttle = throttle;
     addRequirements(Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.set(0.5*Foreward.getAsDouble()*Throttle.getAsDouble()+1/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
