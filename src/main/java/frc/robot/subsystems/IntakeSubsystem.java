// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.MOTOR_IDS.INTAKE.INTAKE_CAN_ID, MotorType.kBrushless);
  private DigitalInput intakeSensor = new DigitalInput(5);

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false); //change as needed
  }

  public Command feedCommand(){
    return new InstantCommand(() -> intakeMotor.set(0.1)).until(this::noNote);
  }

  public Command outTake(){
    return new InstantCommand(() -> intakeMotor.set(-0.8));
  }

  public Command runIntake(){
    return new InstantCommand(() -> intakeMotor.set(0.8)).until(this::hasNote).andThen(new InstantCommand(() -> intakeMotor.set(0)));
  }

  public boolean noNote(){
    return !intakeSensor.get();
  }

  public boolean hasNote(){
    return intakeSensor.get();
  }

  @Override
  public void periodic() {

  }
}
