package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.concurrent.ThreadLocalRandom;

public class ConeFaceRecognitionPipeline extends OpenCvPipeline {
    //  recognition result
    String determinedSurfaceColor = "UNRECOGNIZED"; // one of: {"UNRECOGNIZED", "RED", "GREEN", "BLUE"}
    String msg = "";

    // hyper-parameters
    int poleSize = 10000;
    ColorBoundary redColorBoundary = new ColorBoundary(new int[]{100, 255}, new int[]{0, 100}, new int[]{0, 100});
    ColorBoundary greenColorBoundary = new ColorBoundary(new int[]{0, 100}, new int[]{100, 255}, new int[]{0, 100});
    ColorBoundary blueColorBoundary = new ColorBoundary(new int[]{0, 100}, new int[]{0, 100}, new int[]{70, 255});


    // limit the processing field of view
    int rowStart = 180;
    int rowEnd = 220;
    int colStart = 50;
    int colEnd = 150;


    // Define the segmented matrix
    Mat submat;

    @Override
    public void init(Mat firstFrame) {
        submat = firstFrame.submat(rowStart, rowEnd, colStart, colEnd);

        // do the recognition task
        String[] candidates = {"RED", "GREEN", "BLUE"};
        int[] poleResult = {0, 0, 0}; // refers to r, g, b
        double[] pixel = new double[0];
        for (int i = 0; i < poleSize; i++) {

            /*
            row = ThreadLocalRandom.current().nextInt(0, rowEnd - rowStart+ 1);
            col = ThreadLocalRandom.current().nextInt(0, colEnd - colStart + 1);
            */

            int row, col;
            row = ThreadLocalRandom.current().nextInt(rowStart, rowEnd + 1);
            col = ThreadLocalRandom.current().nextInt(colStart, colEnd + 1);
            pixel = firstFrame.get(row, col);


            if (redColorBoundary.isWithinColorBoundary((int) pixel[0], (int) pixel[1], (int) pixel[2])) {
                poleResult[0] += 1;
            }

            if (greenColorBoundary.isWithinColorBoundary((int) pixel[0], (int) pixel[1], (int) pixel[2] )) {
                poleResult[1] += 1;
            }

            if (blueColorBoundary.isWithinColorBoundary((int) pixel[0], (int) pixel[1], (int) pixel[2])) {
                poleResult[2] += 1;
            }
        }
        determinedSurfaceColor = candidates[argmax(poleResult)];
        msg = "" + poleResult[0] + " " + poleResult[1] + " " + poleResult[2];


        submat.get(0, 0);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        return submat;
        //return input;
    }

    public String getDeterminedSurfaceColor() {
        return determinedSurfaceColor;
    }

    public String getMsg() {
        return msg;
    }
    public static int argmax(int[] a) {
        int re = Integer.MIN_VALUE;
        int arg = -1;
        for (int i = 0; i < a.length; i++) {
            if (a[i] > re) {
                re = a[i];
                arg = i;
            }
        }
        return arg;
    }

}
