package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LoopArm implements Action {
    private final Robot robotInstance;

    public LoopArm(Robot robotInstance) {
        this.robotInstance = robotInstance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            robotInstance.handleArm();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return false;
    }




}
