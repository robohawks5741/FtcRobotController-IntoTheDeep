package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.BotConstants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.BotConstants.tagsize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;

public class Utilities {
    //`adb connect 192.168.43.1:5555`
    private static final double MAX_MOTOR_CURRENT = 20  ;
    public double checkPower(DcMotorEx motor) {
        double power = motor.getPower();
        if(motor.getCurrent(CurrentUnit.AMPS) > MAX_MOTOR_CURRENT) {
            return power * MAX_MOTOR_CURRENT / motor.getCurrent(CurrentUnit.AMPS);
        }
        else if(motor.getCurrent(CurrentUnit.AMPS) > MAX_MOTOR_CURRENT * 3 / 4 && !motor.getMode().equals("RUN_WITHOUT_ENCODER")) {
            return 1;
        }
        return power;
    }


    //this is written assuming origin is the center of the field
    //math is done with 0 degrees is pointing away from audience, towards blue net zone and red observation zone
    //and then adjusted for roadrunner axes

    public static Pose2d pointToPose(AprilTagDetection detection, Mat matrix) {
        AprilTagPipeline.Pose D0Fpose = AprilTagPipeline.poseFromTrapezoid(detection.corners, matrix, tagsize, tagsize);

        double xInit = detection.pose.x * INCHES_PER_METER;
        //weirdness between what's considered y and z I think
        double yInit = detection.pose.z * INCHES_PER_METER;
        double yaw = D0Fpose.getRvec().get(2, 0)[0];
        double r = Math.sqrt(Math.pow(xInit, 2) + Math.pow(yInit, 2));
        double headingWithYaw = Math.atan(xInit / yInit);
        //the bot's heading relative to the wall
        double trueHeading = headingWithYaw + yaw;
        //reorient with tag position
        //there's very likely to be some sign error/90 degree off angle here
        trueHeading = -trueHeading + Math.toRadians(TagConstants.TagPositions.getA(detection.id));
        //not sure about any of this to be honest I'm just trying stuff
        //this should work I think--it's disgusting but I don't feel like wading through all this trig to find
        //one sign error
        double trueX, trueY;
        if(detection.id == 12 || detection.id == 15) {
            trueX = r * Math.cos(trueHeading) + TagConstants.TagPositions.getX(detection.id);
            trueY = -r * Math.sin(trueHeading) + TagConstants.TagPositions.getY(detection.id);
        }
        else {
            trueX = -r * Math.cos(trueHeading) + TagConstants.TagPositions.getX(detection.id);
            trueY = r * Math.sin(trueHeading) + TagConstants.TagPositions.getY(detection.id);
        }
        double z = -detection.pose.y * INCHES_PER_METER + TagConstants.TagPositions.getZ(detection.id);
        return new Pose2d(trueX, trueY, trueHeading + Math.PI);
    }
}
