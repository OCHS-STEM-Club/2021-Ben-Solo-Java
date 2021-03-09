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

}
