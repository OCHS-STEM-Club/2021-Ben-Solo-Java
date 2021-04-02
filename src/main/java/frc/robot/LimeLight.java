package frc.robot;

import edu.wpi.first.networktables.*;

public class LimeLight {

    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry networkTableEntry;
    NetworkTableInstance networkTableInstance;

    public boolean courseAFirstRedBallIsThere() {
        double xValue = getTargetOffsetAngleHorizontal();
        double yValue = getTargetOffsetAngleVertical();
        boolean horizontalRange = xValue >= -2 && xValue <= 6;
        boolean verticalRange = yValue >= -14 && yValue <= -6;
        boolean targetRange = horizontalRange && verticalRange;
        return targetRange;
    }
    public boolean courseBFirstRedBallIsThere() {
        double xValue = getTargetOffsetAngleHorizontal();
        double yValue = getTargetOffsetAngleVertical();
        boolean horizontalRange = xValue >= -22 && xValue <= -14;
        boolean verticalRange = yValue >= -14 && yValue <= -6;
        boolean targetRange = horizontalRange && verticalRange;
        return targetRange;
    }


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
