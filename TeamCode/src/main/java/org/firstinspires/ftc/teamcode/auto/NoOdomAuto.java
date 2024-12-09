package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Timer;

@Autonomous(name = "BasicAuto")
public class NoOdomAuto extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ElapsedTime t = new ElapsedTime();
        double timePassed = t.milliseconds();
        waitForStart();
        while(opModeIsActive()) {
            if(t.milliseconds() < timePassed + 2000) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0, -1
                        ),
                        0
                ));
            }
        }
    }
}

