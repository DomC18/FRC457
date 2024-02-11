// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armLeader = new CANSparkMax(Constants.MOTOR_IDS.ARM.ARMRIGHT_CAN_ID, MotorType.kBrushless);
  private SparkPIDController armPIDController = armLeader.getPIDController();
  private SparkAbsoluteEncoder armEncoder = armLeader.getAbsoluteEncoder(Type.kDutyCycle);

  private CANSparkMax armFollower = new CANSparkMax(Constants.MOTOR_IDS.ARM.ARMLEFT_FOLLOWER_CAN_ID, MotorType.kBrushless);

  public ArmSubsystem() {
    armLeader.restoreFactoryDefaults();
    armFollower.restoreFactoryDefaults();
    armLeader.setInverted(false); //change as needed
    armFollower.setInverted(false); //change as needed
    armFollower.follow(armLeader);
    
    armEncoder.setPositionConversionFactor(360);

    armPIDController.setFeedbackDevice(armEncoder);
    armPIDController.setP(0); //change as needed
    armPIDController.setI(0); //change as needed
    armPIDController.setD(0); //change as needed
    armPIDController.setFF(0); //change as needed
  }

  public Command armManualDown(){
    return new InstantCommand(() -> armLeader.set(-0.3)); //change speed as needed
  }

  public Command armManualUp(){
    return new InstantCommand(() -> armLeader.set(0.3)); //change speed as needed
  }

  public boolean armAtDesiredPosition(double desiredPosition){
    return (Math.abs(armEncoder.getPosition() - desiredPosition) < 5); //change error as needed
  }

  public Command armGoToPosition(double desiredPosition){
    return new InstantCommand(() -> armPIDController.setReference(desiredPosition, ControlType.kPosition));
  }

  @Override
  public void periodic() {
    
  }
}
