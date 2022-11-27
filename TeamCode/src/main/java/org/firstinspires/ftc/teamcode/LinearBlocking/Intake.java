package org.firstinspires.ftc.teamcode.LinearBlocking;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Intake {
    private final HardwareMap hwMap;

    private final DcMotor motor;
    private final RevColorSensorV3 color;
    private State mode = State.IDLE;

    public Intake(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.motor = hwMap.dcMotor.get("intake");
        this.color = hwMap.get(RevColorSensorV3.class, "color");
    }

    public boolean coneDetected() {
        NormalizedRGBA colorVals = color.getNormalizedColors();
        return colorVals.red > 100 || colorVals.blue > 100;
    }

    enum State {
        IDLE, INTAKE, OUTTAKE;
    }

    public void update() {
        switch (mode) {
            case IDLE:
                this.motor.setPower(0);
                break;

            case INTAKE:
                this.motor.setPower(1);
                break;

            case OUTTAKE:
                this.motor.setPower(-1);
                break;
        }
    }

    public void idle() {
        this.mode = State.IDLE;
    }

    public void intake() {
        this.mode = State.INTAKE;
    }

    public void outtake() {
        this.mode = State.OUTTAKE;
    }
}
