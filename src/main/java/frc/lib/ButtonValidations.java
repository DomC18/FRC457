// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.XboxController;

public class ButtonValidations {
    public boolean getLeftTrigger(XboxController controller){
        double rawValue = controller.getLeftTriggerAxis();
        return (rawValue > 0.05); //change deadzone as needed
    }

    public boolean getRightTrigger(XboxController controller){
        double rawValue = controller.getRightTriggerAxis();
        return (rawValue > 0.05); //change deadzone as needed
    }

    public boolean getLeftStickUp(XboxController controller){
        double rawValue = controller.getLeftY();
        return (rawValue > 0.1); //change deadzone as needed
    }

    public boolean getRightStickUp(XboxController controller){
        double rawValue = controller.getRightY();
        return (rawValue > 0.1); //change deadzone as needed
    }
    
    public boolean getLeftStickDown(XboxController controller){
        double rawValue = controller.getLeftY();
        return (rawValue < -0.1); //change deadzone as needed
    }

    public boolean getRightStickDown(XboxController controller){
        double rawValue = controller.getRightY();
        return (rawValue < -0.1); //change deadzone as needed
    }
}