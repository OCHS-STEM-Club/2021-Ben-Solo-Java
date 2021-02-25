package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BenSoloMotorSetup extends DriveMotorSetup {

    private CANSparkMax driveMotorLeft = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax driveMotorRight = new CANSparkMax(3, MotorType.kBrushless);

    private CANSparkMax slaveMotorLeft1 = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax slaveMotorLeft2 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax slaveMotorRight1 = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax slaveMotorRight2 = new CANSparkMax(7, MotorType.kBrushless);
    private CANEncoder leftCanEncoder = driveMotorLeft.getEncoder();
    private CANEncoder rightCanEncoder = driveMotorRight.getEncoder();
    private static final double GEAR_RATIO = (60d / 11d) * (50d / 38d);

    public BenSoloMotorSetup() {

        slaveMotorLeft1.follow(driveMotorLeft, false);
        slaveMotorLeft2.follow(driveMotorLeft, false);
        slaveMotorRight1.follow(driveMotorRight, false);
        slaveMotorRight2.follow(driveMotorRight, false);

        driveMotorLeft.setSmartCurrentLimit(60);
        driveMotorRight.setSmartCurrentLimit(60);
        slaveMotorLeft1.setSmartCurrentLimit(60);
        slaveMotorRight1.setSmartCurrentLimit(60);
        slaveMotorLeft2.setSmartCurrentLimit(60);
        slaveMotorRight2.setSmartCurrentLimit(60);

        driveMotorLeft.setOpenLoopRampRate(0.35);
        driveMotorRight.setOpenLoopRampRate(0.35);
        slaveMotorLeft1.setOpenLoopRampRate(0.35);
        slaveMotorRight1.setOpenLoopRampRate(0.35);
        slaveMotorLeft2.setOpenLoopRampRate(0.35);
        slaveMotorRight2.setOpenLoopRampRate(0.35);

        CANError lerr = leftCanEncoder.setPositionConversionFactor(1 / GEAR_RATIO * 6 * Math.PI);
        CANError rerr = rightCanEncoder.setPositionConversionFactor(1 / GEAR_RATIO * 6 * Math.PI);

        SmartDashboard.putBoolean("encoder conversion error", lerr != CANError.kOk || rerr != CANError.kOk);

    }

    @Override
    public SpeedController getLeftMotorController() {
        return driveMotorLeft;
    }

    @Override
    public SpeedController getRightMotorController() {
        return driveMotorRight;
    }

    public CANEncoder getLeftCanEncoder() {
        return leftCanEncoder;
    }

    public CANEncoder getRightCanEncoder() {
        return rightCanEncoder;
    }

    @Override
    public double getLeftEncoderInches() {
        return leftCanEncoder.getPosition();

    }

    @Override
    public double getRightEncoderInches() {
        return rightCanEncoder.getPosition();
    }

}