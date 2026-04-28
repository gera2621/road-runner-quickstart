package org.firstinspires.ftc.teamcode.team.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoGate {
    public Servo gate;

    static boolean isGateOpen = false;

    public ServoGate(Servo gate) {
        this.gate = gate;
    }

    public void toggleGate() {
        if (isGateOpen) {
            isGateOpen = false;
            gate.setPosition(0.18);
        } else {
            isGateOpen = true;
            gate.setPosition(0.5);
        }
    }
    public void openGate() {
        isGateOpen = true;
        gate.setPosition(0.22);
    }

    public Action openGateAction() {
        return new InstantAction(
                this::openGate
        );
    }
    public void closeGate() {
        isGateOpen = false;
        gate.setPosition(0.4);
    }

    public Action closeGateAction() {
        return new InstantAction(
                this::closeGate
        );
    }
}
