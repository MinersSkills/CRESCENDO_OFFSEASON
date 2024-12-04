// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeCommand;
// import frc.robot.commands.Shutter.Shutter;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.LEDBlinker;
import frc.robot.subsystems.Autonomo.Shutter;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{ 
  private SendableChooser<Command> autonomousChooser; // Lista de autonomos

  // public CANSparkMax sharedMotor = new CANSparkMax(12, MotorType.kBrushless); //Motor Compartilhado entre as classes
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));

  private LEDBlinker ledBlinker = new LEDBlinker();
  public Intake intake = new Intake(10, 12); // Classe do Intake
  public Shooter shooter = new Shooter(11,13, drivebase); // Classe do Shooter
  
  public Shutter shutter = new Shutter(shooter, intake);
  public IntakeCommand intakeCommand = new IntakeCommand(intake, ledBlinker);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS4Controller drivePS4 = new CommandPS4Controller(0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
    // Exemplo básico de criação de InstantCommand com Runnable
    NamedCommands.registerCommand("startIntake", new InstantCommand(intakeCommand::execute));
    // NamedCommands.registerCommand("stopIntake", new InstantCommand(intakeCommand::end));
    // NamedCommands.registerCommand("stopIntake", new InstantCommand(innerShooter::stopIntake));
    // NamedCommands.registerCommand("startShutter2AMP", new InstantCommand(innerShooter::startShutter2AMP));
    // NamedCommands.registerCommand("setDispAMP", new InstantCommand(innerShooter::setDispAMP));
    // NamedCommands.registerCommand("setStopAMP", new InstantCommand(innerShooter::setStopAMP));
    // NamedCommands.registerCommand("setDispSPK", new InstantCommand(innerShooter::setDispSPK));
    // NamedCommands.registerCommand("setStopSPK", new InstantCommand(innerShooter::setStopSPK));
    // NamedCommands.registerCommand("startShutter2SPK", new InstantCommand(innerShooter::startShutter2SPK));
    // NamedCommands.registerCommand("Tiro SPK", new InstantCommand(shooter.handleAutomaticShooting()));

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autonomousChooser);
    
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(drivePS4.getLeftY() == 0.0 ? Shooter.adjusted : drivePS4.getLeftY()  ,
                                                                                               OperatorConstants.LEFT_Y_DEADBAND ),
                                                                   () -> MathUtil.applyDeadband(drivePS4.getLeftX() == 0.0 ? Shooter.adjustedX : drivePS4.getLeftX()  ,
                                                                                               OperatorConstants.LEFT_X_DEADBAND ),
                                                                   () -> MathUtil.applyDeadband(drivePS4.getRightX() == 0.0 ? Shooter.adjustedR : drivePS4.getRightX() ,
                                                                                                 OperatorConstants.RIGHT_X_DEADBAND ),
                                                                   drivePS4.getHID()::getTouchpad,
                                                                   drivePS4.getHID()::getTouchpad,
                                                                   drivePS4.getHID()::getTouchpad,
                                                                   drivePS4.getHID()::getTouchpad);

                                                                   

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-drivePS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-drivePS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -drivePS4.getRightX(),
        () -> -drivePS4.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(drivePS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) / 1.5,
    //     () -> MathUtil.applyDeadband(drivePS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) / 1.5,
    //     () -> drivePS4.getRightX());

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(drivePS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(drivePS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> drivePS4.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? closedAbsoluteDriveAdv : driveFieldOrientedDirectAngle);
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


    drivePS4.R1().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    // drivePS4.L3().onTrue(NamedCommands)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autonomousChooser.getSelected();
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
