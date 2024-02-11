// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterLeader = new CANSparkMax(Constants.MOTOR_IDS.SHOOTER.SHOOTER_CAN_ID, MotorType.kBrushless);
  private RelativeEncoder shooterEncoder = shooterLeader.getEncoder();
  private SparkPIDController shooterPIDController = shooterLeader.getPIDController();
  private CANSparkMax shooterFollower = new CANSparkMax(Constants.MOTOR_IDS.SHOOTER.SHOOTER_FOLLOWER_CAN_ID, MotorType.kBrushless);

  public ShooterSubsystem() {
    shooterLeader.restoreFactoryDefaults();
    shooterFollower.restoreFactoryDefaults();
    shooterLeader.setInverted(false); //change as needed
    shooterFollower.setInverted(true); //change as needed
    shooterFollower.follow(shooterLeader);

    shooterEncoder.setVelocityConversionFactor(0);

    shooterPIDController.setFeedbackDevice(shooterEncoder);
    shooterPIDController.setP(0); //change as needed
    shooterPIDController.setI(0); //change as needed
    shooterPIDController.setD(0); //change as needed
    shooterPIDController.setFF(0); //change as needed
  }

  public boolean shooterAtSpeed(double desiredSpeed){
    return Math.abs(shooterEncoder.getVelocity()-desiredSpeed) < 100;
  }

  public Command runShooter(double desiredSpeed){
    return new InstantCommand(() -> shooterPIDController.setReference(desiredSpeed, ControlType.kVelocity));
  }

  @Override
  public void periodic() {

  }
}
