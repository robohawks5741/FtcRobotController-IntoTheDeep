package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class SetArmPos implements Action {
    private Robot robotInstance;

    private double arm;
    private double lift;

    private int apos;

    public SetArmPos(Robot robotInstance, double a, double l, int i) {
        this.robotInstance = robotInstance;
        this.arm = a;
        this.lift = l;
        this.apos = i;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            robotInstance.resetPid();
            robotInstance.pressed = true;
            robotInstance.rotateToPosition = arm;
            robotInstance.extendToPosition = lift;
            robotInstance.armPosition = apos;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return false;
    }




}
