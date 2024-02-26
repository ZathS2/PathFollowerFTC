package com.example.linearregressionpos;

import java.io.BufferedReader;
import java.io.FileReader;

public class LinearRegressionPosition
{
    public static void main(String[] args) {

        System.out.println("COMEÇO");
        try(BufferedReader br = new BufferedReader(new FileReader("./main/java/com/example/linearregressionpos/res/sample.txt"))) {
            System.out.println(br.readLine());
        } catch (Exception e)
        {
            System.out.println("ERRO");
        }
    }
}