// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
    //TalonSRX shooterRight;
    CANSparkMax shooterRight;

    CANSparkMax shooterLeft;

  public Shooter() {
   // shooter = new TalonSRX(6);
   //shooterRight = new TalonSRX(6);
    shooterLeft = new CANSparkMax(18, MotorType.kBrushless);

   shooterRight = new CANSparkMax(17, MotorType.kBrushless);
   shooterRight.setInverted(true);
   //shooterLeft.setIdleMode(shooterLeft.IdleMode.kCoast);
   shooterLeft.setIdleMode(IdleMode.kBrake);
    shooterRight.setIdleMode(IdleMode.kBrake);






  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double speed){
  shooterLeft.set(speed);
  shooterRight.set(speed);
  }
}
