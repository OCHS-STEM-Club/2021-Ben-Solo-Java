package frc.robot;

public class RadialSegment {

    private double heading, distance, radius;
    private boolean left;
    
    private RadialSegment(double heading, double distance, double radius, boolean left){
        this.heading= heading;
        this.distance= distance;
        this.radius= radius;
        this.left= left;
    }

    /*
    public static RadialSegment straight(double distance){
        return new RadialSegment(heading, distance, radius, left)
    }*/


}




