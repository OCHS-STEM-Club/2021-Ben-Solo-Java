package frc.robot;

public class SavedPaths {

    public static ContinuousPath.Segment[] MAIN = new ContinuousPath.Segment[] {
        ContinuousPath.Segment.straight(120),
        ContinuousPath.Segment.turn(360, 30),
        ContinuousPath.Segment.straight(90),
        ContinuousPath.Segment.turn(45, 30)
    };

}
