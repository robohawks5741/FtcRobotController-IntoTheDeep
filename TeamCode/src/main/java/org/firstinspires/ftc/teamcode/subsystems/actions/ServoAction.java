package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAction implements Action {
    Servo servo;
    double position;

    public ServoAction(Servo s, double p){
        this.servo = s;
        this.position = p;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket){
        servo.setPosition(position);
        return false;
    }
}