package frc.robot;

public class SavedPaths {

    public static LinearSegment[] BARREL = new LinearSegment[] {
        new LinearSegment(0.00, 150.00),
        new LinearSegment(90.00, 60.00),
        new LinearSegment(180.00, 60.00),
        new LinearSegment(270.00, 60.00),
        new LinearSegment(360.00, 150.00),
        new LinearSegment(270.00, 60.00),
        new LinearSegment(180.00, 60.00),
        new LinearSegment(90.00, 60.00),
        new LinearSegment(45.00, 84.85),
        new LinearSegment(0.00, 60.00),
        new LinearSegment(-90.00, 60.00),
        new LinearSegment(-180.00, 310.00)
    };

    public static LinearSegment[] SLALOM = new LinearSegment[] {
        new LinearSegment(0.00, 60.00),
        new LinearSegment(-90.00, 60.00),
        new LinearSegment(0.00, 180.00),
        new LinearSegment(90.00, 60.00),
        new LinearSegment(0.00, 60.00),
        new LinearSegment(-90.00, 60.00),
        new LinearSegment(-180.00, 60.00),
        new LinearSegment(-270.00, 60.00),
        new LinearSegment(-180.00, 180.00),
        new LinearSegment(-90.00, 60.00),
        new LinearSegment(-180.00, 60.00)
    };

    public static LinearSegment[] BOUNCE = new LinearSegment[] {
        new LinearSegment(0.00, 45.00),
        new LinearSegment(-75.96, 61.85),
        new LinearSegment(73.74, 125.00),
        new LinearSegment(0.00, 45.00),
        new LinearSegment(-85.24, 120.42),
        new LinearSegment(85.24, 120.42),
        new LinearSegment(0.00, 75.00),
        new LinearSegment(-87.61, 120.10),
        new LinearSegment(63.43, 67.08)
    };



    /*
    public static ContinuousPath.Segment[] MAIN = new ContinuousPath.Segment[] {
        ContinuousPath.Segment.straight(30),
        ContinuousPath.Segment.turn(-90, -30),
        ContinuousPath.Segment.turn(0, 30),
        ContinuousPath.Segment.straight(120),
        ContinuousPath.Segment.turn(90, 30),
        ContinuousPath.Segment.turn(-270, -30),        
        ContinuousPath.Segment.turn(-180, 30),
        ContinuousPath.Segment.straight(120),
        ContinuousPath.Segment.turn(-90, 30),
        ContinuousPath.Segment.turn(-180, -30),
        ContinuousPath.Segment.straight(30)
    };


    public static RadialSegment[] MAIN2 = new RadialSegment[] {
        RadialSegment.straight(0, 30),
        RadialSegment.turning(-90, 30, true),
        RadialSegment.turning(0, 30, false),
        RadialSegment.straight(0, 120),
        RadialSegment.turning(90, 30, false),
        RadialSegment.turning(-270, 30, true),
        RadialSegment.turning(-180, 30, false),
        RadialSegment.straight(-180, 120),
        RadialSegment.turning(-90, 30, false),
        RadialSegment.turning(-180, 30, true),
        RadialSegment.straight(-180, 30)
    };
    */

   
    public static LinearSegment[] A1 = new LinearSegment[] {
        new LinearSegment(0.00, 75.00),
        new LinearSegment(26.57, 67.08),
        new LinearSegment(-83.66, 90.55),
        new LinearSegment(21.16, 166.21),
        new LinearSegment(0.00, 30.00)
    };

    public static LinearSegment[] A2 = new LinearSegment[] {
        new LinearSegment(0.00, 30.00),
        new LinearSegment(27.55, 129.71),
        new LinearSegment(-60.95, 102.96),
        new LinearSegment(26.57, 67.08),
        new LinearSegment(0.00, 75.00)
    };

    public static LinearSegment[] B1 = new LinearSegment[] {
        new LinearSegment(0.00, 30.00),
        new LinearSegment(-33.69, 54.08),
        new LinearSegment(45.00, 84.85),
        new LinearSegment(-45.00, 84.85),
        new LinearSegment(15.95, 109.20),
        new LinearSegment(0.00, 30.00)
    };

    public static LinearSegment[] B2 = new LinearSegment[] {
        new LinearSegment(0.00, 30.00),
        new LinearSegment(12.53, 138.29),
        new LinearSegment(-45.00, 84.85),
        new LinearSegment(45.00, 84.85),
        new LinearSegment(-63.43, 33.54),
        new LinearSegment(0.00, 30.00)
    };
}
