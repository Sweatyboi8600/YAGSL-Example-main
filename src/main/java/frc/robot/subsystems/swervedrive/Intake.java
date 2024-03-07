// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
  TalonSRX upIntake, downIntake;
  double voltage;
  public Intake() {
    upIntake = new TalonSRX(15);
    downIntake = new TalonSRX(14);
  }

  @Override
  public void periodic() {
    voltage  = voltage + upIntake.getMotorOutputVoltage();
    // This method will be called once per scheduler run
  }
  public void set(double speed){
    upIntake.set(ControlMode.PercentOutput, speed);
    downIntake.set(ControlMode.PercentOutput, speed);

    

  }
  public double getMotorOutputVoltage(){
    return voltage;
  }
}
