// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceAuto extends SequentialCommandGroup {
  public OnePieceAuto(ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      new ParallelCommandGroup(
        arm.armGoToPosition(Constants.ARM_POSITIONS.subWooferPosition).until(() -> arm.armAtDesiredPosition(Constants.ARM_POSITIONS.subWooferPosition)),
        shooter.runShooter(Constants.SHOOTER_SPEEDS.subWooferSpeed).until(() -> shooter.shooterAtSpeed(Constants.SHOOTER_SPEEDS.subWooferSpeed))
      ),
      intake.feedCommand()
    );
  }
}
