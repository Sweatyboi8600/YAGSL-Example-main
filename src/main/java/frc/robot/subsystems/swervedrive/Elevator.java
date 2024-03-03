// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  TalonSRX leftElevator, rihgtElevator;
  /** Creates a new Elevator. */
  public Elevator() {
    leftElevator = new TalonSRX(20);
        rihgtElevator = new TalonSRX(21);
        leftElevator.setInverted(true);
        leftElevator.setNeutralMode(NeutralMode.Brake);
        rihgtElevator.setNeutralMode(NeutralMode.Brake);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double speed){
    rihgtElevator.set(ControlMode.PercentOutput, speed);
    leftElevator.set(ControlMode.PercentOutput, speed);


  }
}
