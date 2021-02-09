package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
public abstract class DriveMotorSetup {

    public abstract SpeedController getLeftMotorController();

    public abstract SpeedController getRightMotorController();

}
