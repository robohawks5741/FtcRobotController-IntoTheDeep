package org.firstinspires.ftc.teamcode.subsystems;

public final class TagConstants {
    public static final class TagPositions{

        //Tag positions are from Blue side up with 0 degrees pointing right
        //Currently 0 degrees is facing away from audience
        public final Pose TAG11 = new Pose(-70.72081, 47.125, 5.29, 0);
        public final Pose TAG12 = new Pose(0, 70.616, 5.29, 270);
        public final Pose TAG13 = new Pose(70.61631, 47.125, 5.29, 180);

        public final Pose TAG14 = new Pose(70.61631, -47.125, 5.29, 180);
        public final Pose TAG15 = new Pose(0, -70.616, 5.29, 90);
        public final Pose TAG16 = new Pose(-70.61631, -47.125, 5.29, 0);


    }
    public static TagPositions TAG_POSITIONS = new TagPositions();

     public static class Pose{
        double x;
        double y;
        double z;
        double angle;
        public Pose(double x,double y,double z, double a){
            this.x = x;
            this.y = y;
            this.z = z;
            angle = a;
        }

        public Pose Diff(Pose other){
            return (new Pose(other.x-x, other.y-y, other.z-z, other.angle - angle));
        }
        public double x() {
           return x;
        }
        public double y() {
            return y;
        }
        public double z() {
            return z;
        }
        public double angle() {
            return angle;
        }
    }
}
