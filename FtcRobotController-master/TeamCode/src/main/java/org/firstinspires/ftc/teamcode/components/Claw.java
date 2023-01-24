package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends BaseComponent {

    public Servo leftServo;
    public Servo rightServo;


    public ModeSwitcher servoPositions = new ModeSwitcher(new double[]{0, 0.3}, 0);

    public Claw(Servo _leftServo, Servo _rightServo) {
        leftServo = _leftServo;
        rightServo = _rightServo;
    }

    public void togglePosition() {
        servoPositions.recursiveIncrement();
    }

    public double getCurrentPosition() {
        return servoPositions.getValue();
    }
    public void run(double runtime) {
        leftServo.setPosition(servoPositions.getValue());
        rightServo.setPosition(-servoPositions.getValue());
    }

}
