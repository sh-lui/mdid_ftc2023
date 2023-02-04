
package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends BaseComponent {

    public Servo leftServo;
    public Servo rightServo;

    private double MIN_POSITION = 0.02;
    private double MAX_POSITION = 0.65;

    private double currentPosition = 0;
    private double armRaiseRate = -0.07;
    private double coupling_offset = +0.025;

    public Arm(Servo _leftServo, Servo _rightServo) {
        leftServo =  _leftServo;
        rightServo = _rightServo;
    }

    public void run(double runtime) {
        leftServo.setPosition(currentPosition);
        rightServo.setPosition(1+coupling_offset - currentPosition);
    }

    public void incrementPosition( double triggerValue) {
        currentPosition = Math.max(MIN_POSITION, Math.min( currentPosition + triggerValue * armRaiseRate, MAX_POSITION));
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

}
