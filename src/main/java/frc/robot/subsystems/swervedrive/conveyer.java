// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class conveyer extends SubsystemBase {
  TalonSRX topConveryer, bottomConveyer; 
  /** Creates a new conveyer. */
  public conveyer() {
    topConveryer= new TalonSRX(16);
    //bottomConveyer = new TalonSRX(0);
    //topConveryer.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void conveyrSet (double speed)
  {
    topConveryer.set(ControlMode.PercentOutput, -speed);
   // bottomConveyer.set(ControlMode.PercentOutput, -speed);


  }
  
}
