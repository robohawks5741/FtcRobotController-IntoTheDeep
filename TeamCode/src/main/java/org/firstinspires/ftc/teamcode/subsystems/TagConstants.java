package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

public final class TagConstants {
    public static final class TagPositions{

        //Tag positions are from Blue side up with 0 degrees pointing right, origin is center
        //Currently 0 degrees is facing away from audience
        //in inches I think
        public static final Pose TAG11 = new Pose(-70.72081, 47.125, 5.29, 0);
        public static final Pose TAG12 = new Pose(0, 70.616, 5.29, 270);
        public static final Pose TAG13 = new Pose(70.61631, 47.125, 5.29, 180);

        public static final Pose TAG14 = new Pose(70.61631, -47.125, 5.29, 180);
        public static final Pose TAG15 = new Pose(0, -70.616, 5.29, 90);
        public static final Pose TAG16 = new Pose(-70.61631, -47.125, 5.29, 0);

        public static double getX(int id) {
            switch(id) {
                case 11:
                    return TAG11.x();
                case 12:
                    return TAG12.x();
                case 13:
                    return TAG13.x();
                case 14:
                    return TAG14.x();
                case 15:
                    return TAG15.x();
                case 16:
                    return TAG16.x();
                default:
                    return 0;
            }
        }
        public static double getY(int id) {
            switch(id) {
                case 11:
                    return TAG11.y();
                case 12:
                    return TAG12.y();
                case 13:
                    return TAG13.y();
                case 14:
                    return TAG14.y();
                case 15:
                    return TAG15.y();
                case 16:
                    return TAG16.y();
                default:
                    return 0;
            }
        }
        public static double getZ(int id) {
            switch(id) {
                case 11:
                    return TAG11.z();
                case 12:
                    return TAG12.z();
                case 13:
                    return TAG13.z();
                case 14:
                    return TAG14.z();
                case 15:
                    return TAG15.z();
                case 16:
                    return TAG16.z();
                default:
                    return 0;
            }
        }
        public static double getA(int id) {
            switch(id) {
                case 11:
                    return TAG11.angle();
                case 12:
                    return TAG12.angle();
                case 13:
                    return TAG13.angle();
                case 14:
                    return TAG14.angle();
                case 15:
                    return TAG15.angle();
                case 16:
                    return TAG16.angle();
                default:
                    return 0;
            }
        }
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
        //copy constructor
        public Pose(Pose other) {
            this.x = other.x;
            this.y = other.y;
            this.z = other.z;
            this.angle = other.angle;
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
        @NonNull
        @Override
        public String toString() {
            return x + ", " + y + ", " + z + ", " + angle;
        }
    }
}
