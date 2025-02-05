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
import org.firstinspires.ftc.teamcode.subsystems.actions.LoopArm;
import org.firstinspires.ftc.teamcode.subsystems.actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.SetArmPos;

@Autonomous(name = "Auto Left")
public class AutoLeft extends AutoSuper {


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        super.init();

        waitForStart();
        if (opModeIsActive()){


            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .stopAndAdd(new SetArmPos(this, BotConstants.ARM_FRONT_PLACING_VOLTS, BotConstants.LIFT_EXTENDED_VOLTS))
                            .splineTo(new Vector2d(11.1396, 22.0307), Math.toRadians(-45.0))
                            .build(),
                    new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED),
                    new LoopArm(this)
            ));



        }

    }

}