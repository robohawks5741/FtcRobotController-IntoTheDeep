package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LoopArm implements Action {
    private final Robot robotInstance;
    private ElapsedTime timer;
    public LoopArm(Robot robotInstance) {
        timer = new ElapsedTime();
        this.robotInstance = robotInstance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            while (!robotInstance.stopArm){
                robotInstance.handleArm();
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        telemetryPacket.addLine(String.format("Arm positions: %f", robotInstance.armPosition));
        return timer.seconds() > 1;
    }




}
