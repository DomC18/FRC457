// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceAutoBlue extends SequentialCommandGroup {
  public ThreePieceAutoBlue(ArmSubsystem arm, DriveSubystem drive, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.subWooferPosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.subWooferPosition)),
        shooter.runShooter(Constants.SHOOTER_SPEEDS.subWooferSpeed).until(() -> shooter.shooterAtSpeed(Constants.SHOOTER_SPEEDS.subWooferSpeed))
      ),
      intake.feedCommand(),

      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.intakePosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.intakePosition)),
        intake.runIntake(),
        drive.driveForward().until(() -> drive.robotAtPoint(new Translation2d(0, 0))) //change point here
      ),

      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.podiumPosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.podiumPosition)),
        shooter.runShooter(Constants.SHOOTER_SPEEDS.podiumSpeed).until(() -> shooter.shooterAtSpeed(Constants.SHOOTER_SPEEDS.podiumSpeed))
      ),
      intake.feedCommand(),

      drive.turnLeft().until(() -> drive.robotAtRotation(new Rotation2d(90))),
      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.intakePosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.intakePosition)),
        intake.runIntake(),
        drive.driveForward().until(() -> drive.robotAtPoint(new Translation2d(0, 0))) //change point here
      ),

      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.podiumPosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.podiumPosition)),
        shooter.runShooter(Constants.SHOOTER_SPEEDS.podiumSpeed).until(() -> shooter.shooterAtSpeed(Constants.SHOOTER_SPEEDS.podiumSpeed)),
        drive.turnRight().until(() -> drive.robotAtRotation(new Rotation2d(45)))
      ),
      
      intake.feedCommand()
    );
  }
}
