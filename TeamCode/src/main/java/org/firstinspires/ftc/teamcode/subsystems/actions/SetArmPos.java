package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class SetArmPos implements Action {
    private Robot robotInstance;
    private int apos;

    public SetArmPos(Robot robotInstance, int i) {
        this.robotInstance = robotInstance;
        this.apos = i;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            robotInstance.resetPid();
            robotInstance.pressed = true;
            robotInstance.armPosition = apos;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return false;
    }




}
