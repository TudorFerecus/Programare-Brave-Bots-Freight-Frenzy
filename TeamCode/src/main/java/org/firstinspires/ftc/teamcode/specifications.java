package org.firstinspires.ftc.teamcode;

public class specifications
{
    // robot's speed
    public static double moving_speed = 0.8;

    // intake speed
    public static double intake_speed = 1;

    //viteza de tragere/eliminare a obiectelor din cuva
    public static double motor_cuva_speed = 1;

    //de cate tick-uri trece un encoder cand robotul se misca un inch
    public static double counts_per_inch = 1103.88170;

    //pentru calibrarea robotului, viteza cu care se roteste robotul in jurul axei sale pentru calibrare
    public static double pivot_speed = 0.2;

    //delay intre calcule pozitie odometrie
    public static int odometry_delay = 75;

    //marja eroare pid
    public static int pid_eroare_coord = 10;
    public static int pid_eroare_unghi = 2;

    public static int total_miscari = 2;
}