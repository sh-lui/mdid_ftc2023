package org.firstinspires.ftc.teamcode.vision;

public class ColorBoundary {
    // the arrays consists of two numbers: {min, max}
    private int[] redBoundaryArr;
    private int[] greenBoundaryArr;
    private int[] blueBoundaryArr;

    public ColorBoundary(int[] _redBoundaryArr, int[] _greenBoundaryArr, int[] _blueBoundaryArr) {
        redBoundaryArr = _redBoundaryArr;
        greenBoundaryArr = _greenBoundaryArr;
        blueBoundaryArr = _blueBoundaryArr;
    }

    public int getRedMin() {
        return redBoundaryArr[0];
    }
    public int getRedMax() {
        return redBoundaryArr[1];
    }

    public int getGreenMin() {
        return greenBoundaryArr[0];
    }
    public int getGreenMax() {
        return greenBoundaryArr[1];
    }

    public int getBlueMin() {
        return blueBoundaryArr[0];
    }
    public int getBlueMax() {
        return blueBoundaryArr[1];
    }

    public boolean isWithinColorBoundary(int R, int G, int B) {
        boolean isWithinRedBoundary = R < getRedMax() && R > getRedMin();
        boolean isWithinGreenBoundary = G < getGreenMax() && G > getGreenMin();
        boolean isWithinBlueBoundary = B < getBlueMax() && B > getBlueMin();
        return isWithinRedBoundary && isWithinGreenBoundary && isWithinBlueBoundary;
    }
}

