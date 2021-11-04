package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private int pozRata = -1;

    private boolean gasitRata = false;

    private double timpInit = -1;


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
        if(!gasitRata) {
            go(specifications.poz_rata_1[0], specifications.poz_rata_1[1], specifications.poz_rata_1[2], 0);
            cautaRata(1, 1);
            if (pozRata == -1) {
                go(specifications.poz_rata_2[0], specifications.poz_rata_2[1], specifications.poz_rata_2[2], 2);
                cautaRata(3, 2);
            }
            if (pozRata == -1) {
                go(specifications.poz_rata_3[0], specifications.poz_rata_3[1], specifications.poz_rata_3[2], 4);
                cautaRata(5, 3);
            }
            gasitRata = true;
        }
        if(gasitRata)
        {
            go(specifications.poz_wobble[0], specifications.poz_wobble[1], specifications.poz_wobble[2], 6);
            ridicareBrat(pozRata, 7);
            punereObiect(8);
        }
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

    private void cautaRata(int index_miscare, int poz)
    {
        if(moment_miscare == index_miscare)
        {
            double distance = hardware.senzorRata.getDistance(DistanceUnit.CM);
            if (distance <= specifications.distantaMaxRata)
            {
                pozRata = poz;
            }
            moment_miscare += 1;
        }
    }

    //ridicare brat la un nivel predefinit
    //presupunem ca bratul pleaca de la nivelul 1
    private void ridicareBrat(int pozitie, int index_miscare)
    {
        if(index_miscare == moment_miscare)
        {
            if(timpInit == -1)
                timpInit = runTime.milliseconds();

            if (pozitie == 2 && timpInit > runTime.milliseconds() - specifications.time_position_2)
                hardware.motorCuva.setPower(specifications.motor_cuva_speed);

            else if(pozitie == 3 && timpInit > runTime.milliseconds() - specifications.time_position_3)
                hardware.motorCuva.setPower(specifications.motor_cuva_speed);

            if((timpInit < runTime.milliseconds() - specifications.time_position_3 && pozitie == 3) ||
                    (timpInit < runTime.milliseconds() - specifications.time_position_3 && pozitie == 3))
            {
                hardware.motorCuva.setPower(0);
                moment_miscare += 1;
            }

        }
    }

    //lasare obiect din cuva
    private void punereObiect(int index_miscare)
    {
        if(index_miscare == moment_miscare)
        {
            if(timpInit == -1)
                timpInit = runTime.milliseconds();
            hardware.motorCuva.setDirection(DcMotorSimple.Direction.REVERSE); //reverse pt a lasa obiectul

            if(timpInit > runTime.milliseconds() - specifications.outtake_time)
                hardware.motorCuva.setPower(specifications.intake_speed);

            else
            {
                hardware.motorCuva.setPower(0);
                moment_miscare += 1;
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