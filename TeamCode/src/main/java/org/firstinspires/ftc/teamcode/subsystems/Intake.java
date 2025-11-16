package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.LED;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;
    private final ServoEx latch;
    private final RevBlinkinLedDriver led;

    public Intake(final HardwareMap hMap) {
        intake = new MotorEx(hMap, "intake");
        latch = new ServoEx(hMap, "latch");
//        led = new ServoEx(hMap, "led");
        led = hMap.get(RevBlinkinLedDriver.class, "led");
        intake.setRunMode(MotorEx.RunMode.RawPower);
        intake.set(0);
    }

    public Command collect() {
        return new InstantCommand(() -> intake.set(1));
    }

    public Command reverse() {
        return new InstantCommand(() -> intake.set(-1));
    }

    public Command stop() {
        return new InstantCommand(() -> intake.set(0));
    }

    public Command open() {
        return new InstantCommand(() -> latch.set(1));
    }

    public Command close() {
        return new InstantCommand(() -> latch.set(0));
    }

    public Command LEDon() {
        return new InstantCommand(() -> led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN));
    }

    public Command LEDoff() {
        return new InstantCommand(() -> led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE));
    }
}