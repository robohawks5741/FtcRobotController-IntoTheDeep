package org.firstinspires.ftc.teamcode.auto;
import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.actions.ServoAction;

@Autonomous(name = "Auto Left")
public class AutoLeft extends AutoSuper {


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        init();

        waitForStart();
        if (opModeIsActive()){

            runBlocking(
                    drive.actionBuilder(beginPose)
                    .setTangent( Math.toRadians(-45.0))
                    .splineToLinearHeading(new Pose2d(11.1396, 22.0307,  Math.toRadians(-45.0)), 0)

                    .stopAndAdd(new ServoAction(clawIntake, BotConstants.CLAW_CLOSED))
                    .build()
            );

        }
    }

}
