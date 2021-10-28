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

    @Override
    public void init()
    {
        //init hardware
        hardware = new hardware(hardwareMap);
    }

    @Override
    public void start()
    {
        runTime.reset();
    }

    @Override
    public void loop()
    {

    }
}
