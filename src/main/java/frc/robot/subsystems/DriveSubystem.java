// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubystem extends SubsystemBase {
  private CANSparkMax leftMain = new CANSparkMax(Constants.MOTOR_IDS.DRIVE.LEFT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(Constants.MOTOR_IDS.DRIVE.LEFT_FOLLOWER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightMain = new CANSparkMax(Constants.MOTOR_IDS.DRIVE.RIGHT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(Constants.MOTOR_IDS.DRIVE.RIGHT_FOLLOWER_CAN_ID, MotorType.kBrushless);
  private RelativeEncoder leftEncoder = leftMain.getEncoder();
  private RelativeEncoder rightEncoder = rightMain.getEncoder();
  private SparkPIDController leftPIDController = leftMain.getPIDController();
  private SparkPIDController rightPIDController = rightMain.getPIDController();

  public AHRS navx;
  double currentX;
  double currentY;
  double currentYaw;

  DifferentialDrive drive;
  
  public DriveSubystem() {
    leftMain.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightMain.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftMain.setInverted(false); //change as needed
    leftFollower.setInverted(false); //change as needed
    rightMain.setInverted(false); //change as needed
    rightFollower.setInverted(false); //change as needed
    leftFollower.follow(leftMain);
    rightFollower.follow(rightMain);

    leftEncoder.setPositionConversionFactor(6*Units.inchesToMeters(6));
    rightEncoder.setPositionConversionFactor(6*Units.inchesToMeters(6));

    leftPIDController.setFeedbackDevice(leftEncoder);
    rightPIDController.setFeedbackDevice(rightEncoder);
    leftPIDController.setP(0); //change as needed
    rightPIDController.setP(0); //change as needed
    leftPIDController.setI(0); //change as needed
    rightPIDController.setI(0); //change as needed
    leftPIDController.setD(0); //change as needed
    rightPIDController.setD(0); //change as needed

    navx = new AHRS(Port.kMXP);

    drive = new DifferentialDrive(leftMain, rightMain);
  }

  public boolean robotAtRotation(Rotation2d rotation){
    return (Math.abs(currentYaw-rotation.getDegrees()) < 5);
  }

  public boolean robotAtPoint(Translation2d point){
    return (
      (Math.abs(currentX-point.getX()) < 0.5) && //change error as needed
      (Math.abs(currentY-point.getY()) < 0.5) //change error as needed
    ); 
  }

  public Command driveForward(){
    return new InstantCommand(() -> drive.tankDrive(0.8, 0.8));
  }

  public Command driveBackward(){
    return new InstantCommand(() -> drive.tankDrive(-0.8, -0.8));
  }

  public Command turnLeft(){
    return new InstantCommand(() -> drive.tankDrive(-0.5, 0.5));
  }
  
  public Command turnRight(){
    return new InstantCommand(() -> drive.tankDrive(0.5, -0.5));
  }

  public Command driveComand(Supplier<Double> leftStick, Supplier<Double> rightStick){
    return new InstantCommand(() -> drive.tankDrive(leftStick.get(), rightStick.get()));
  }

  @Override
  public void periodic() {

  }
}
