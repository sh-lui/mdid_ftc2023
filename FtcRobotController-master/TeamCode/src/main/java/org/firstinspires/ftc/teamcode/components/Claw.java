package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends BaseComponent {

    public Servo leftServo;
    public Servo rightServo;

    public double closePosition = 0.27;
    public double openPosition = 0.315;

    public ModeSwitcher servoPositions = new ModeSwitcher(new double[]{closePosition, openPosition}, 1);

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

    public void open() {
        leftServo.setPosition(openPosition);
        rightServo.setPosition(1-openPosition);
    }
    public void close() {
        leftServo.setPosition(closePosition);
        rightServo.setPosition(1-closePosition);
    }



    public void run(double runtime) {
        leftServo.setPosition(servoPositions.getValue());
        rightServo.setPosition(1-servoPositions.getValue());
    }

}
