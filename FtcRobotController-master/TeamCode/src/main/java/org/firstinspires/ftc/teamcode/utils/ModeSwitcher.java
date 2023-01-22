package org.firstinspires.ftc.teamcode.utils;

public class ModeSwitcher {
    public double[] optionArr = {};
    public int currentIndex = -1;

    public ModeSwitcher(double[] _optionArr, int _currentIndex) {
        currentIndex = _currentIndex;
        optionArr = _optionArr;
    }

    public double getValue() {
        return optionArr[currentIndex];
    }

    public void increment() {
        if (currentIndex < optionArr.length - 1) {
            currentIndex++;
        }
    }

    public void decrement() {
        if (currentIndex > 0) {
            currentIndex--;
        }
    }

    public void recursiveDecrement() {
        if (currentIndex > 0) {
            currentIndex--;
        } else {
            currentIndex = optionArr.length - 1;
        }
    }

    public void recursiveIncrement() {
        if (currentIndex < optionArr.length - 1) {
            currentIndex++;
        } else {
            currentIndex = 0;
        }
    }

}
