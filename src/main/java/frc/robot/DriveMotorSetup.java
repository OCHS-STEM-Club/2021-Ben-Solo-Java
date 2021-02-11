package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
public abstract class DriveMotorSetup {

    public abstract SpeedController getLeftMotorController();

    public abstract SpeedController getRightMotorController();

    public abstract double getLeftEncoderInches();
    public abstract double getRightEncoderInches();
    

}
