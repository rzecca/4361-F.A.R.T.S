package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.RunningOpModes.OLDCODE.FINAL_BLUE_TELEOP;


@TeleOp(name = "AUTOGETTER", group = "Sensor")
public class AUTOGETTER extends LinearOpMode {

    ElapsedTime actionTimer = new ElapsedTime();

    //Driving
    public double FLTargetPower;
    public double FRTargetPower;
    public double BLTargetPower;
    public double BRTargetPower;
    public double Z_Weight_Current;
    public double headingCurrent;

    private VoltageSensor ControlHub_VoltageSensor;
    private VoltageSensor ExpansionHub2_VoltageSensor;

    //pinpoint
    GoBildaPinpointDriver pinpoint;

    //speeds
    double Speed = 1;
    double X;
    double Y;
    double Z;
    double adj_Angle;
    double Z_Weight = 0.75;

    //sticks
    double xstick;
    double ystick;
    double headingstick;

    //on off buttons
    boolean ta;
    boolean sa;
    boolean tb;
    boolean sb;
    boolean txx;
    boolean sx;
    boolean tyy;
    boolean syy;
    boolean sdd;
    boolean tdd;
    boolean sdl;
    boolean tdl;
    boolean sdu;
    boolean tdu;
    boolean sdr;
    boolean tdr;

    boolean stateyy;
    boolean thingyyy;

    //ALL LIMELIGHT PIDS
//    private Limelight3A LL;
    private double disance;
    double tx;
    double ty;
    public static double kP = .05;
    public static double maxPower = 1;
    double theLnumber;


    //launcher PID
    double lastlaunchererror;
    double targetlauncher;
    double LAUNCHERVELO;
    double launchererror;
    double launcherV;

    //    double launchers = Arrays.asList(LLAUNCHER, RLAUNCHER);
    //Turret PID's
    //tune
    public double launcherturretonbearingPconstantvaluethingy = .031;
    public double launcherturretonbearingIconstantvaluethingier = 0;
    public double launcherturrtonbearingDconstantvaluethingiest = 0.0047;
    double intergalSum = 0;
    double stricktness;
    double launcherturretonbearingerror;
    ElapsedTime timer = new ElapsedTime();
    private double lastLauncherturretonbearingerror;

    //rpms
    double ticksPerRev = 28;
    double rpm = 100;
    double ticksPerSec = (rpm * ticksPerRev) / 60.0;


    public void setPinpoint(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    //sequence timer stuff
    enum ActionState {
        IDLE,
        ACTION1,
        WAITING,
        ACTION2
    }
    BIOBUZZv1.ActionState actionState = BIOBUZZv1.ActionState.IDLE;
    //TURRET CODE
    // Target coordinates in inches, using the same map as Pinpoint.
    double TARGET_X = 48;
    double TARGET_Y = -83;

    // 435 RPM goBILDA Yellow Jacket.
    double MOTOR_TICKS_PER_REV = 384.5;

    // Motor output-shaft turns per full turret revolution.
// Direct drive = 1.0.
// Example: 20-tooth motor gear driving 100-tooth turret gear = 5.0.
    double TURRET_GEAR_RATIO = 1.5;

    // +1 if positive motor power and increasing encoder counts turn
// the turret counterclockwise. Otherwise use -1.
    double TURRET_SIGN = 1;

    // Replace these example limits with your turret's actual travel.
// Zero degrees means pointing straight forward.
    double TURRET_MIN_DEG = -180;
    double TURRET_MAX_DEG = 190;

    // Starting values — tune on your robot.
    double TURRET_P = .03;
    double TURRET_D = 0.0002;
    double TURRET_MAX_POWER = .3;
    double TURRET_TOLERANCE_DEG = .05;
    private void aimTurret(DcMotorEx turret) {

        // Your existing loop must call pinpoint.update() first.
        if (pinpoint.getDeviceStatus()
                != GoBildaPinpointDriver.DeviceStatus.READY) {
            turret.setPower(0);
            return;
        }

        double ticksPerTurretRev = MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO;

        if (!Double.isFinite(ticksPerTurretRev) || ticksPerTurretRev <= 0
                || Math.abs(TURRET_SIGN) != 1
                || !Double.isFinite(TURRET_MIN_DEG)
                || !Double.isFinite(TURRET_MAX_DEG)
                || TURRET_MIN_DEG >= TURRET_MAX_DEG
                || !Double.isFinite(TURRET_P) || TURRET_P < 0
                || !Double.isFinite(TURRET_D) || TURRET_D < 0
                || !Double.isFinite(TURRET_MAX_POWER) || TURRET_MAX_POWER <= 0
                || !Double.isFinite(TURRET_TOLERANCE_DEG)
                || TURRET_TOLERANCE_DEG < 0) {
            turret.setPower(0);
            return;
        }
        Pose2D pose = pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);

        // Native Pinpoint heading: counterclockwise positive.
        // Do not use your negated driving heading here.
        double robotHeading = pose.getHeading(AngleUnit.DEGREES);

        // Assumes turret pivot is at Pinpoint's robot tracking point.
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        if (!Double.isFinite(dx) || !Double.isFinite(dy)
                || !Double.isFinite(robotHeading)
                || Math.hypot(dx, dy) < 0.01) {
            turret.setPower(0);
            return;
        }

        double fieldBearing = Math.toDegrees(Math.atan2(dy, dx));

        double relativeBearing =
                ((fieldBearing - robotHeading + 180) % 360 + 360) % 360 - 180;

        double currentAngle = TURRET_SIGN
                * turret.getCurrentPosition()
                * 360.0 / ticksPerTurretRev;

        // Choose an equivalent target inside the turret's travel limits.
        double firstTurn = Math.ceil(
                (TURRET_MIN_DEG - relativeBearing) / 360.0);
        double lastTurn = Math.floor(
                (TURRET_MAX_DEG - relativeBearing) / 360.0);

        if (firstTurn > lastTurn) {
            turret.setPower(0);
            telemetry.addData("Turret", "Target outside travel limits");
            return;
        }

        double nearestTurn = Math.max(firstTurn, Math.min(lastTurn,
                Math.rint((currentAngle - relativeBearing) / 360.0)));

        double targetAngle = relativeBearing + 360.0 * nearestTurn;

        // Do not wrap this error; that could command travel through a stop.
        double error = targetAngle - currentAngle;

        double velocityDegSec = TURRET_SIGN
                * turret.getVelocity()
                * 360.0 / ticksPerTurretRev;

        double power = TURRET_P * error - TURRET_D * velocityDegSec;

        if (!Double.isFinite(power)) {
            turret.setPower(0);
            return;
        }

        double maxPower = Math.min(1, TURRET_MAX_POWER);
        power = Math.max(-maxPower, Math.min(maxPower, power));

        if (Math.abs(error) <= TURRET_TOLERANCE_DEG
                || (currentAngle <= TURRET_MIN_DEG && power < 0)
                || (currentAngle >= TURRET_MAX_DEG && power > 0)) {
            power = 0;
        }

        turret.setPower(TURRET_SIGN * power);

        telemetry.addData("Turret actual angle", currentAngle);
        telemetry.addData("Turret target angle", targetAngle);
        telemetry.addData("Turret error", error);
    }

    private Kitchenet stuff = new Kitchenet(this);


    @Override
    public void runOpMode() throws InterruptedException {

        stuff.initializeAuto();
        stuff.telemetryupdate();


        DcMotorEx FL;
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx BL;
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx FR;
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BR;
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        DcMotorEx intake;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx launcher;
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx turret;
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        Servo ramp;
        ramp = hardwareMap.get(Servo.class, "ramp");
        Servo stopper;
        stopper = hardwareMap.get(Servo.class, "stopper");


//        Limelight3A LL;
//        LL = hardwareMap.get(Limelight3A.class, "LL");
//        Servo light;
//        light = hardwareMap.get(Servo.class, "light");
        //intake current switch
        double INTAKE_CURRENT_LIMIT_AMPS = 9.5; // Tune for your intake.
        double INTAKE_OVERLOAD_DELAY_SEC = 0.2;

        boolean intakeOn = false;
        boolean intakeTripped = false;
        boolean intakeOverCurrent = false;
        boolean lastIntakeA = false;
        boolean lastIntakeReset = false;

        ElapsedTime intakeOverloadTimer = new ElapsedTime();


        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        //reversals
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        //drive motors brake
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


//        LL.start();

        turret.setPower(0);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (opModeIsActive()) {

            //possible turret code
            stuff.telemetryupdate();
        }}}