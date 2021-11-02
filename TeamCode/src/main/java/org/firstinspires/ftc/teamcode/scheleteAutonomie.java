package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autonomie", group="autonomie")
//@Disabled

/*
Miscarea robotului se face pe baza unui index de miscare (miscare = modificarea pozitiei robotului)
    - total_miscari - numarul total de miscari in autonomie
    - moment_miscare - la a cata miscare se afla robotul

Functia go() contine indexul de miscare (a cata miscare este din numarul total de miscari ale robotului)
    - in functie de moment_miscare, va modifica destinatia robotului
    - nu misca robotul, doar seteaza "destinatia" robotului

Functia movement_robot() misca robotul, in fucntie de outputul PID-ului.
    - movement_robot() va fi apelata doar o data, la finalul loop()

 */
abstract class scheleteAutonomie extends OpMode
{
    protected hardware hardware;
    protected ElapsedTime runTime = new ElapsedTime();
    protected odometry odometrie; //stores robot's position on field

    protected int moment_miscare = 1;
    protected int total_miscari;

    //PID components for x, y and angle
    protected PID brainX = null;
    protected PID brainY = null;
    protected PID brainU = null;

    protected void miscare_robot()
    {
        //stop the robot if all robot movements have been completed
        if(total_miscari == moment_miscare)
        {
            for(int i = 0; i < hardware.motor.length; i++)
                hardware.motor[i].setPower(0);
        }
        else movement_pid();
    }

    protected void movement_pid()
    {
        //power for each wheel, dependent of pid output
        double[] power = new double[4];
        power[0] =   + brainX.output(odometrie.getX())*specifications.moving_speed - brainY.output(odometrie.getY())*specifications.moving_speed + brainU.output(odometrie.getAngle());//+
        power[1] =   - brainX.output(odometrie.getX())*specifications.moving_speed - brainY.output(odometrie.getY())*specifications.moving_speed + brainU.output(odometrie.getAngle());//-
        power[2] =   + brainX.output(odometrie.getX())*specifications.moving_speed + brainY.output(odometrie.getY())*specifications.moving_speed + brainU.output(odometrie.getAngle());//-
        power[3] =   - brainX.output(odometrie.getX())*specifications.moving_speed + brainY.output(odometrie.getY())*specifications.moving_speed + brainU.output(odometrie.getAngle());//+
        for(int i=0; i<4; i++)
        {
            hardware.motor[i].setPower(power[i]);
        }
    }

    protected void go(double x, double y, double u, int index_miscare)
    {
        //am ajuns la a n-a miscare
        if(index_miscare == moment_miscare)
        {
            //seteaza destinatia
            brainX.setTarget(x);
            brainY.setTarget(y);
            brainU.setTarget(u);

            //daca s-a ajuns la destinatie, treci la urmatoarea miscare
            if (reachedDestination(x, y))
            {
                brainX.reset();
                brainU.reset();
                brainY.reset();
                moment_miscare++;
            }
        }
    }

    //TODO: funcion should be in PID
    protected boolean reachedDestination(double x, double y)
    {
        if(Math.abs(odometrie.getX()-x) < specifications.pid_tolerance_coord && Math.abs(odometrie.getY()-y) < specifications.pid_tolerance_coord)
            return true;
        return false;
    }
}