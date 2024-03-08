package com.example.linearregressionpos;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class LinearRegressionPosition
{
    public static void main(String[] args)
    {
        List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();

        try
        {
            URL path = LinearRegressionPosition.class.getResource("xSamples.txt");
            File txtFile = new File(path.getFile());
            Scanner reader = new Scanner(txtFile);
            while (reader.hasNext())
                xValues.add(Double.parseDouble(reader.nextLine()));

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        try
        {
            URL path = LinearRegressionPosition.class.getResource("ySamples.txt");
            File txtFile = new File(path.getFile());
            Scanner reader = new Scanner(txtFile);
            while (reader.hasNext())
                yValues.add(Double.parseDouble(reader.nextLine()));

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }


        double ySum = 0.0;
        double xSum = 0.0;
        double xSquareSum = 0.0;
        double xySum = 0.0;

        for (int i = 0; i < xValues.size(); i++)
        {
            double currentX = xValues.get(i);
            xSum += currentX;
            xSquareSum += (currentX * currentX);

            double currentY = yValues.get(i);
            ySum += currentY;
            xySum += (currentY * currentX);
        }

        double n = xValues.size();
        double a = (ySum * xSquareSum - xSum * xySum) / (n * xSquareSum - xSum * xSum);
        double b = (n * xySum - xSum * ySum) / (n * xSquareSum - xSum * xSum);

        System.out.println("a = " + a);
        System.out.println("b = " + b);
    }
}