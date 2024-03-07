package com.example.linearregressionpos;

import java.io.File;
import java.io.FileNotFoundException;
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
            File txtFile = new File("C:\\Users\\Usuario\\Documents\\GitHub\\PathFollowerFTC\\LinearRegressionPos\\src\\main\\java\\com\\example\\linearregressionpos\\res\\xSamples.txt");
            Scanner reader = new Scanner(txtFile);
            while (reader.hasNext())
                xValues.add(Double.parseDouble(reader.nextLine()));

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        try
        {
            File txtFile = new File("C:\\Users\\Usuario\\Documents\\GitHub\\PathFollowerFTC\\LinearRegressionPos\\src\\main\\java\\com\\example\\linearregressionpos\\res\\ySamples.txt");
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