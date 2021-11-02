package org.firstinspires.ftc.teamcode;

public class specifications
{
    public static double moving_speed = 0.8; //viteza robot
    public static double intake_speed = 1;
    public static double motor_cuva_speed = 1;

    //variabile pentru odometrie
    public static double raza_roata_odm = 5; // in cm
    public static double L = 20; //distanta dintre encoderele paralele
    public static double B = 5;  //distanta dintre encoderul din spate si mijlocul dintre celelalte 2 encodere paralele
    public static double ticks_per_revolution = 8192;
    public static double cm_per_tick = 2 * Math.PI * raza_roata_odm / ticks_per_revolution;

    public static int odometry_delay = 75; //delay in odometrie

    //marja eroare pid
    public static int pid_tolerance_coord = 10;
    public static int pid_tolernace_unghi = 2;
}
