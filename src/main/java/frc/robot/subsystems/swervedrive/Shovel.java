// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shovel extends SubsystemBase {
  TalonSRX shovel;

  /** Creates a new Shovel. */
  public Shovel() {
    shovel = new TalonSRX(19);
    shovel.setInverted(false);
    shovel.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double speed){
   shovel.set(ControlMode.PercentOutput, speed);
  }
}
