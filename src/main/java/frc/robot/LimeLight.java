package frc.robot;

import edu.wpi.first.networktables.*;

public class LimeLight {

    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry networkTableEntry;
    NetworkTableInstance networkTableInstance;



    public double getTargetOffsetAngleHorizontal() {
        return limeLightTable.getEntry("tx").getDouble(0.0);
    } 

    public double getTargetOffsetAngleVertical() {
        return limeLightTable.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return limeLightTable.getEntry("ta").getDouble(0.0);
    }

    public double getTargetSkew() {
        return limeLightTable.getEntry("ts").getDouble(0.0);
    }

}
