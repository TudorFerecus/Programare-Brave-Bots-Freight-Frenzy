package org.firstinspires.ftc.teamcode;

public class autonomieA extends scheleteAutonomie {

    @Override
    public void init()
    {
        //init hardware
        hardware = new hardware(hardwareMap);

        total_miscari = 2;

        //init odometrie and thread
        odometrie = new odometry(0,0,0,hardware, specifications.odometry_delay);
        Thread odometrieThread = new Thread(odometrie);
        odometrieThread.start();

        //init PID components
        brainX = new PID(0.7, 0, 0);
        brainY = new PID(1, 0, 0);
        brainU = new PID(1.5, 0, 0);

        //set tolerance
        brainX.setTolerance(specifications.pid_tolerance_coord);
        brainY.setTolerance(specifications.pid_tolerance_coord);
        brainU.setTolerance(specifications.pid_tolernace_unghi);

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
    public void loop() {
        go(5, 10 , 90, 1);
        go(10, 75, 65, 2);

        miscare_robot();
    }
}
