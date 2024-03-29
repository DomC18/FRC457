// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.ButtonValidations;
import frc.robot.autos.OnePieceAuto;
import frc.robot.autos.ThreePieceAutoBlue;
import frc.robot.autos.ThreePieceAutoRed;
import frc.robot.autos.TwoPieceAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
*/
public class RobotContainer {
  private XboxController driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  private XboxController operator = new XboxController(Constants.OperatorConstants.kOperatorControllerPort);
  private ButtonValidations buttonValidator = new ButtonValidations();

  private ArmSubsystem arm = new ArmSubsystem();
  private DriveSubystem drive = new DriveSubystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    autoChooser.addOption("1 Note Auto", new OnePieceAuto(arm, intake, shooter));
    autoChooser.addOption("2 Note Auto", new TwoPieceAuto(arm, drive, intake, shooter));
    autoChooser.addOption("3 Note Auto Blue", new ThreePieceAutoBlue(arm, drive, intake, shooter));
    autoChooser.addOption("3 Note Auto Red", new ThreePieceAutoRed(arm, drive, intake, shooter));
    autoChooser.setDefaultOption("1 Note Auto", new OnePieceAuto(arm, intake, shooter)); //if you want a default auto

    configureBindings();
  }

  private void configureBindings() {
    //Driver Controls
    drive.setDefaultCommand(drive.driveComand(driver::getLeftY, driver::getRightY));
    new JoystickButton(driver, XboxController.Button.kBack.value).whileTrue(intake.feedCommand());
    

    //Operator Controls
    new Trigger(() -> buttonValidator.getLeftStickUp(operator)).whileTrue(intake.runIntake());
    new Trigger(() -> buttonValidator.getLeftStickDown(operator)).whileTrue(intake.outTake());
    new Trigger(() -> buttonValidator.getRightStickUp(operator)).whileTrue(arm.armManualUp());
    new Trigger(() -> buttonValidator.getRightStickDown(operator)).whileTrue(arm.armManualDown());
    new POVButton(operator, 0).onTrue(shooter.runShooter(Constants.SHOOTER_SPEEDS.ampSpeed));
    new JoystickButton(operator, XboxController.Button.kA.value).onTrue(arm.armGoToPosition(Constants.ARM_POSITIONS.subWooferPosition));
    new JoystickButton(operator, XboxController.Button.kX.value).onTrue(arm.armGoToPosition(Constants.ARM_POSITIONS.podiumPosition));
    new JoystickButton(operator, XboxController.Button.kB.value).onTrue(arm.armGoToPosition(Constants.ARM_POSITIONS.midPosition));
    new JoystickButton(operator, XboxController.Button.kY.value).onTrue(arm.armGoToPosition(Constants.ARM_POSITIONS.ampPosition));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).onTrue(shooter.runShooter(Constants.SHOOTER_SPEEDS.podiumSpeed));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(intake.feedCommand());
    new Trigger(() -> buttonValidator.getLeftTrigger(operator)).onTrue(shooter.runShooter(Constants.SHOOTER_SPEEDS.subWooferSpeed));
    new Trigger(() -> buttonValidator.getRightTrigger(operator)).onTrue(shooter.runShooter(Constants.SHOOTER_SPEEDS.midSpeed));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
