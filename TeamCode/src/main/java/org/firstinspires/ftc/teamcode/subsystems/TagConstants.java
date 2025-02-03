package org.firstinspires.ftc.teamcode.subsystems;

public final class TagConstants {
    public final class TagPositions{

        //Tag positions are from Blue side up with 0 degrees pointing right
        public static final Pose TAG11 = new Pose(-70.72081, 47.125, 5.29);
        public final Pose TAG12 = new Pose(0, 70.616, 5.29);
        public final Pose TAG13 = new Pose(70.61631, 47.125, 5.29);

        public final Pose TAG14 = new Pose(70.61631, -47.125, 5.29);
        public final Pose TAG15 = new Pose(0, -70.616, 5.29);
        public final Pose TAG16 = new Pose(-70.61631, -47.125, 5.29);


    }

     public class Pose{
        double x;
        double y;
        double z;
        public Pose(double x,double y,double z){
            this.x = x;
            this.y= y;
            this.z = z;
        }

        public Pose Diff(Pose other){
            return (new Pose(other.x-x, other.y-y, other.z-z));
        }
    }
}
