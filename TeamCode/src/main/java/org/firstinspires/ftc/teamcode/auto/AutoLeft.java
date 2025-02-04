package org.firstinspires.ftc.teamcode.auto;
import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;


import com.acmerobotics.roadrunner.ftc.Actions;
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
        while (opModeIsActive()){


            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(11.1396, 22.0307), Math.toRadians(-45.0))
                            .stopAndAdd(new ServoAction(clawIntake, BotConstants.CLAW_CLOSED))
                            .build(),
                    new ServoAction(clawIntake, 0)
            ));



        }

    }

}