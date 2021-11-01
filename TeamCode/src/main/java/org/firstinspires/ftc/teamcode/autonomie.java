package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autonomie", group="autonomie")
//@Disabled
public class autonomie extends OpMode
{
    private hardware hardware;
    private ElapsedTime runTime = new ElapsedTime();
    private odometryGlobalCoordinatePosition odometrie;

    private int moment_miscare = 0;

    private PID brainX = null;
    private PID brainY = null;
    private PID brainU = null;

    private double globalX, globalY, globalU;

    @Override
    public void init()
    {
        //init hardware
        hardware = new hardware(hardwareMap);

        //init odometrie and thread
        odometrie = new odometryGlobalCoordinatePosition(hardware.encStanga, hardware.encDreapta, hardware.encOrizontal,
                                                         specifications.counts_per_inch, specifications.odometry_delay);
        Thread odometrieThread = new Thread(odometrie);
        odometrieThread.start();

        //init PID components
        brainX = new PID(0.7, 0, 0);
        brainY = new PID(1, 0, 0);
        brainU = new PID(1.5, 0, 0);

        //set tolerance
        brainX.setTolerance(specifications.pid_eroare_coord);
        brainY.setTolerance(specifications.pid_eroare_coord);
        brainU.setTolerance(specifications.pid_eroare_unghi);

        brainX.reset();
        brainY.reset();
        brainU.reset();
    }

    @Override
    public void start()
    {
        runTime.reset();
    }

    @Override
    public void loop()
    {
        //get robot position
        globalX = odometrie.returnXCoordinate();
        globalY = odometrie.returnYCoordinate();
        globalU = odometrie.returnOrientation();

        //verifica pozitie rata / obiect
        go(100, 100 ,0, 0);


        miscare_robot();
    }

    private void miscare_robot()
    {
        if(specifications.total_miscari == moment_miscare)
        {
            for(int i = 0; i < hardware.motor.length; i++)
                hardware.motor[i].setPower(0);
        }
        else movement_pid();
    }

    private void movement_pid()
    {
        double[] power = new double[4];
        power[0] =   + brainX.output(globalX)*specifications.moving_speed - brainY.output(globalY)*specifications.moving_speed + brainU.output(globalU);//+
        power[1] =   - brainX.output(globalX)*specifications.moving_speed - brainY.output(globalY)*specifications.moving_speed + brainU.output(globalU);//-
        power[2] =   + brainX.output(globalX)*specifications.moving_speed + brainY.output(globalY)*specifications.moving_speed + brainU.output(globalU);//-
        power[3] =   - brainX.output(globalX)*specifications.moving_speed + brainY.output(globalY)*specifications.moving_speed + brainU.output(globalU);//+
        for(int i=0; i<4; i++)
        {
            hardware.motor[i].setPower(power[i]);
        }
    }

    private void go(double x, double y, double u, int index_miscare)
    {
        if(index_miscare == moment_miscare)
        {
            brainX.setTarget(x);
            brainY.setTarget(y);
            brainU.setTarget(u);
            if (donexy(x, y))
            {
                brainX.reset();
                brainU.reset();
                brainY.reset();
                moment_miscare++;
            }
        }
    }

    private boolean donexy(double x, double y)
    {
        if(Math.abs(globalX-x) < specifications.pid_eroare_coord && Math.abs(globalY-y) < specifications.pid_eroare_coord)
            return true;
        return false;
    }
}