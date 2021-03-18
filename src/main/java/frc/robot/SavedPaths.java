package frc.robot;

public class SavedPaths {

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
}
