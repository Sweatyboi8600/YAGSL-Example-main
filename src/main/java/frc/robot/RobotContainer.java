// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.IntakeAndConveyUp;
import frc.robot.commands.swervedrive.ConveyDown;
import frc.robot.commands.swervedrive.IntakeAndConveyUp;
import frc.robot.commands.swervedrive.ConveyUp;
import frc.robot.commands.swervedrive.ElevateDown;
import frc.robot.commands.swervedrive.ElevateUp;
import frc.robot.commands.swervedrive.Expel;
import frc.robot.commands.swervedrive.Shoot;
import frc.robot.commands.swervedrive.ShootLow;
import frc.robot.commands.swervedrive.ShootMax;
import frc.robot.commands.swervedrive.Shoveling;
import frc.robot.commands.swervedrive.Vore;
import frc.robot.commands.swervedrive.shovelDown;
//import frc.robot.commands.swervedrive.auto.AutoIntake;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.IntakeAndConveyDown;
import frc.robot.subsystems.swervedrive.Elevator;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.Shovel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.conveyer;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
   private final conveyer Conveyer = new conveyer();
  private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Shovel shovel = new Shovel();
    private final Elevator elevator = new Elevator();
    //private ShootMax = new ShootMax(shooter).until(() -> shooter.getMotorOutputVoltage()>= (6));
Joystick js = new Joystick(1); // 0 is the USB Port to be used as indicated on the Driver Station

   public Trigger conveyUpTrigger, conveyDownTrigger, voreTrigger, expelTrigger, shootMaxTrigger, 
   shootLow, ShovelingTrigger, ShovelDownTrigger, ElevateUpTrigger, ElevateDownTrigger,
   ShootOverrideTrigger, ConveyandIntakeUp, ConveyandIntakeDown;  



  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("ConveyIntake", new IntakeAndConveyUp(intake, Conveyer).until(() -> intake.getMotorOutputVoltage() <=(-150)));
    NamedCommands.registerCommand("ShootMax", new ShootMax(shooter).until(() -> shooter.getBusVoltage() <= (-150)));
    NamedCommands.registerCommand("ConveyShoot",new ConveyUp(Conveyer).until(() -> Conveyer.getMotorOutputVoltage() <= (-150)));

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
        // SmartDashboard.putNumber("Joystick X value", shootLow.getApplied);

          //shooter.setDefaultCommand(new Shoot(shooter, () -> js.getRawAxis(1)));

        //driveFieldOrientedAnglularVelocity
        //driveFieldOrientedDirectAngle
        //closedAbsoluteDriveAdv
        

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
     shootMaxTrigger = new JoystickButton(js, 1).whileTrue(new ShootMax(shooter));
    // voreTrigger = new JoystickButton(js, 2).whileTrue(new Vore(intake));
     ShootOverrideTrigger =  new JoystickButton(js, 3).whileTrue(new Shoot(shooter, 
     () -> js.getRawAxis(1), () -> js.getRawAxis(3)));
     ConveyandIntakeUp = new JoystickButton(js, 11).whileTrue(new IntakeAndConveyUp(intake, Conveyer));
     ConveyandIntakeDown = new JoystickButton(js, 7).whileTrue(new IntakeAndConveyDown(intake, Conveyer));
     //expelTrigger = new JoystickButton(js, 7).whileTrue(new Expel(intake));
     conveyDownTrigger = new JoystickButton(js, 8).whileTrue(new ConveyDown(Conveyer));
     ElevateDownTrigger = new JoystickButton(js, 9).whileTrue(new ElevateDown(elevator));
     ElevateUpTrigger = new JoystickButton(js, 10).whileTrue(new ElevateUp(elevator));
     shootLow = new JoystickButton(js, 12).whileTrue(new ShootLow(shooter));


     












   // conveyUpTrigger = new JoystickButton(js, 11).whileTrue(new ConveyUp(Conveyer));
    //conveyDownTrigger = new JoystickButton(js, 8).whileTrue(new ConveyDown(Conveyer));
    //voreTrigger = new JoystickButton(js, 2).whileTrue(new Vore(intake));
    //expelTrigger = new JoystickButton(js, 7).whileTrue(new Expel(intake));
    //ShovelDownTrigger = new JoystickButton(js, 5).whileTrue(new Shoveling(shovel));
    //ShovelingTrigger = new JoystickButton(js, 7).whileTrue(new shovelDown(shovel));
    //ElevateUpTrigger = new JoystickButton(js, 10).whileTrue(new ElevateUp(elevator));
   // ElevateDownTrigger = new JoystickButton(js, 9).whileTrue(new ElevateDown(elevator));
    //shootLow = new JoystickButton(js, 12).whileTrue(new ShootLow(shooter));
    //shootMaxTrigger = new JoystickButton(js, 1).whileTrue(new ShootMax(shooter));
    //ShootOverrideTrigger =  new JoystickButton(js, 3).whileTrue(new Shoot(shooter, 
     //() -> js.getRawAxis(1), () -> js.getRawAxis(3)));
    // ConveyandIntakeUp = new JoystickButton(js, 4).whileTrue(new ConveyUp(Conveyer));
    // ConveyandIntakeUp = new JoystickButton(js, 4).whileTrue(new Vore(intake));
   // ConveyandIntakeDown = new JoystickButton(js, 5).whileTrue(new IntakeAndConveyDown(intake, Conveyer));
     //ConveyandIntakeDown = new JoystickButton(js, 5).whileTrue(new ConveyUp(Conveyer));
    // ConveyandIntakeUp = new JoystickButton(js, 4).whileTrue(new IntakeAndConveyUp(intake, Conveyer));



     
     // new Vore(intake));// new Vore(intake));

   
  

    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // SequentialCommandGroup auto = new SequentialCommandGroup(
    //  //new Vore(intake).until(() -> intake.getMotorOutputVoltage()<=(-150)) ),
    //  new IntakeAndConveyUp(intake, Conveyer).until(() -> intake.getMotorOutputVoltage() <=(-150)),
    //  new ShootMax(shooter).until(() -> shooter.getCanBusVoltage() <= (-150)),
    //  new ConveyUp(Conveyer).until(() -> Conveyer.getMotorOutputVoltage() <= (-150)));

      //new AutoIntake(intake));

   //  new Vore(intake).until(() -> intake.getMotorOutputVoltage()>=(6)));
   //   new AutoIntake(intake));
    ;
      
    //return auto;
  

    //return auto;
    //
   //new Vore(intake).until(() -> intake.getMotorOutputVoltage()>=(6));

    //new ShootMax(shooter).until(() -> shooter.getaa)

   return new PathPlannerAuto("Test");}
  
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
  

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
