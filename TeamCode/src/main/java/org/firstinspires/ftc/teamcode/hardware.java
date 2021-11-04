package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardware
{

    public DcMotor[] motor = new DcMotor[4]; // motoarele rotilor

    public DcMotor motorCarusel = null; // motor carusel

    public DcMotor motorIntake = null; // motor intake

    public DcMotor motorCuva = null; // motor cuva

    // encodere odometrie
    public DcMotor encStanga;
    public DcMotor encDreapta;
    public DcMotor encOrizontal;

    // parametrii imu
    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;

    public Servo brat = null;
    public Servo gheara = null;

    private HardwareMap hardwareMap = null;

    // senzor rata
    public DistanceSensor senzorRata = null;

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        senzorRata = hardwareMap.get(DistanceSensor.class, "senzor");

        initMotors();
        initImu();
    }

    private void initImu()
    {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
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

        // encodere odometire (acleasi nume ca primele 3 motoare)
        setDefaultStateMotor(encStanga, "m0", DcMotorSimple.Direction.FORWARD);
        setDefaultStateMotor(encDreapta, "m1", DcMotorSimple.Direction.FORWARD);
        setDefaultStateMotor(encOrizontal, "m3", DcMotorSimple.Direction.FORWARD);

        // motoare roti
        for (int i = 0; i < motor.length; i++)
        {
            String nume = "m" + Integer.toString(i);
            setDefaultStateMotor(motor[i], nume, DcMotorSimple.Direction.REVERSE);
        }
    }

}