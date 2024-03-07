// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.conveyer;
import frc.robot.subsystems.swervedrive.Intake;



public class IntakeAndConveyUp extends Command {
  private final Intake intake;
  private final conveyer conveyer;
  /** Creates a new IntakeAndConveyUp. */
  public IntakeAndConveyUp(Intake intake, conveyer conveyer) {
    this.intake = intake; 
    this.conveyer = conveyer;
     addRequirements(intake,conveyer); //here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(-1);
    conveyer.set(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    conveyer.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
