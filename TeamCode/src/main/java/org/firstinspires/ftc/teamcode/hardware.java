package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardware
{
    // motoarele rotilor
    public DcMotor[] motor = new DcMotor[4];

    // motor carusel
    public DcMotor motorCarusel = null;

    // motor intake
    public DcMotor motorIntake = null;

    // motor cuva
    public DcMotor motorCuva = null;


    public Servo brat = null;
    public Servo gheara = null;

    private HardwareMap hardwareMap = null;

    public ElapsedTime runTime = new ElapsedTime();

    public hardware(HardwareMap hdMap)
    {
        initializare(hdMap);
    }

    private void initializare(HardwareMap hdMap)
    {
        this.hardwareMap = hdMap;
        brat = hardwareMap.get(Servo.class, "brat");
        gheara = hardwareMap.get(Servo.class, "gheara");

        initMotors();
    }

    private void setDefaultStateMotor(DcMotor motor, String nume, DcMotorSimple.Direction direction)
    {
        motor = hardwareMap.get(DcMotor.class, nume);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        motor.setDirection(direction);
    }

    private void initMotors()
    {
        // motor cuva
        setDefaultStateMotor(motorCuva, "mcuva", DcMotorSimple.Direction.FORWARD);

        // motor brat intake
        setDefaultStateMotor(motorIntake, "mintake", DcMotorSimple.Direction.FORWARD);

        // motor brat rata
        setDefaultStateMotor(motorCarusel, "mcarusel", DcMotorSimple.Direction.FORWARD);

        // motoare roti
        for (int i = 0; i < motor.length; i++)
        {
            String nume = "m" + Integer.toString(i);
            setDefaultStateMotor(motor[i], nume, DcMotorSimple.Direction.REVERSE);
        }
    }

}
