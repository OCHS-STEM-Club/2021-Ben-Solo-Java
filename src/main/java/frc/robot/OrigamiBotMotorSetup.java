package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SpeedController;

public class OrigamiBotMotorSetup extends DriveMotorSetup {

    private WPI_TalonSRX driveMotorLeft = new WPI_TalonSRX(1);
    private WPI_TalonSRX driveMotorRight = new WPI_TalonSRX(1);
    private WPI_TalonSRX slaveMotorRight1 = new WPI_TalonSRX(1);
    private WPI_TalonSRX slaveMotorRight2 = new WPI_TalonSRX(1);
    private WPI_TalonSRX slaveMotorLeft1 = new WPI_TalonSRX(1);
    private WPI_TalonSRX slaveMotorLeft2 = new WPI_TalonSRX(1);

    private SpeedControllerGroup leftGroup = new SpeedControllerGroup(driveMotorLeft, slaveMotorLeft1, slaveMotorLeft2);
    private SpeedControllerGroup rightGroup = new SpeedControllerGroup(driveMotorRight, slaveMotorRight1,
            slaveMotorRight2);

    public SpeedController getLeftMotorController() {
        return leftGroup;
    }

    public SpeedController getRightMotorController() {
        return rightGroup;
    }

    @Override
    public double getLeftPositionInches() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightPositionInches() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLeftVelocityInchesPerSecond() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightVelocityInchesPerSecond() {
        // TODO Auto-generated method stub
        return 0;
    }

}