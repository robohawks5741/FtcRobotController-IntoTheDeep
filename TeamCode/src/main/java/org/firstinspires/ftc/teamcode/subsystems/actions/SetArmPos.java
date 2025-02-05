package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SetArmPos implements Action {
    private Robot robotInstance;

    private double arm;
    private double lift;

    public SetArmPos(Robot robotInstance, double a, double l) {
        this.robotInstance = robotInstance;
        this.arm = a;
        this.lift = l;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            robotInstance.resetPid();
            robotInstance.pressed = true;
            robotInstance.armPosition = 5;
            robotInstance.lift.resetPID();
            robotInstance.rotate.resetPID();
            robotInstance.rotateToPosition = arm;
            robotInstance.extendToPosition = lift;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return false;
    }




}
