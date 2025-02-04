package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

public class SetArmPos implements Action {
    DualMotor rotate;
    DualMotor lift;
    double rotateTarget;
    double liftTarget;
    public SetArmPos(DualMotor rotate, double rotateTarget, DualMotor lift, double liftTarget) {
        this.rotate = rotate;
        this.rotateTarget = rotateTarget;
        this.lift = lift;
        this.liftTarget = liftTarget;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }
}
