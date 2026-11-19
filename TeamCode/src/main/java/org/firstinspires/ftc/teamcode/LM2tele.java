package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.RunningOpModes.OLDCODE.FINAL_BLUE_TELEOP;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "LM2 Tele-op", group = "Sensor")
public class LM2tele extends LinearOpMode {

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
    LM2tele.ActionState actionState = LM2tele.ActionState.IDLE;
    //TURRET CODE
    // Target coordinates in inches, using the same map as Pinpoint.
    public static double TARGET_X = 57.5;
    public static double TARGET_Y = -52.5;
    public static double launcherP = .65;
    public static double launcherD = .002;




    public static double setlaunchervelo;

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
    double TURRET_MIN_DEG = -200;
    double TURRET_MAX_DEG = 200;

    // Starting values — tune on your robot.
    double TURRET_P = .03;
    double TURRET_D = 0.0002;
    double TURRET_MAX_POWER = .3;
    double TURRET_TOLERANCE_DEG = .05;


    @Override
    public void runOpMode() throws InterruptedException {
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
        pinpoint.setOffsets(-3.085, -3.42, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
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

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


//        LL.start();

        turret.setPower(0);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean lastReferenceButton = false;
        boolean aimAtRightGoal = false;

        waitForStart();


        while (opModeIsActive()) {
            pinpoint.update();

            boolean referenceButton = gamepad1.back;

// Reset once each time BACK is pressed.
            if (referenceButton && !lastReferenceButton) {
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH, 0, 0,
                        AngleUnit.DEGREES, 0
                ));
            }

            lastReferenceButton = referenceButton;


            double turretDegrees =
                    turret.getCurrentPosition() * 360.0 / 579.0;

            double stick = -gamepad2.right_stick_x;

            if (Math.abs(stick) < 0.08) {
                stick = 0;
            }

            double power;

            if (opModeIsActive()) {
                double robotX = pinpoint.getPosX(DistanceUnit.INCH);
                double robotY = pinpoint.getPosY(DistanceUnit.INCH);
                double robotHeading = pinpoint.getHeading(AngleUnit.DEGREES);

                // Switch goals after crossing midfield.
// The buffer prevents repeated switching near the line.
                if (robotY < -72.7) {
                    aimAtRightGoal = true;
                } else if (robotY > -66.7) {
                    aimAtRightGoal = false;
                }

// Both goals use the same coordinates as the corner reset.
                double targetX = 50;
                double targetY = aimAtRightGoal ? -90.0 : -52.5;

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                telemetry.addData("Selected goal",
                        aimAtRightGoal ? "Right" : "Left");

                double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

                // Convert field direction into an angle relative to the robot.
                double relativeAngle = fieldAngle - robotHeading;

                // Put that direction between -180 and +180 degrees.
                relativeAngle = Math.toDegrees(Math.atan2(
                        Math.sin(Math.toRadians(relativeAngle)),
                        Math.cos(Math.toRadians(relativeAngle))));

                // Find an equivalent angle inside the cable limits.
                // Leave 10 degrees of extra room at either end.
                double targetAngle = Double.NaN;
                double shortestMove = Double.POSITIVE_INFINITY;

                for (int turn = -1; turn <= 1; turn++) {
                    double candidate = relativeAngle + turn * 360.0;

                    if (candidate >= -230 && candidate <= 195) {
                        double move = Math.abs(candidate - turretDegrees);

                        if (move < shortestMove) {
                            shortestMove = move;
                            targetAngle = candidate;
                        }
                    }
                }

                power = 0;

                // Avoid aiming when sitting directly on the target coordinate.
                if (!Double.isNaN(targetAngle) && Math.hypot(dx, dy) > 1.0) {
                    double error = targetAngle - turretDegrees;

                    power = error * 0.005;
                    power = Math.max(-0.60, Math.min(0.60, power));

                    if (Math.abs(error) < 2.0) {
                        power = 0;
                    }

                    telemetry.addData("Turret target", "%.1f degrees", targetAngle);
                    telemetry.addData("Aim error", "%.1f degrees", error);
                }
            } else {
                power = stick * 0.15;
            }

// Slow outward movement near the cable limits.
            if (power > 0) {
                double scale = Math.max(0, Math.min(1,
                        (205.0 - turretDegrees) / 40.0));
                power *= scale;
            } else if (power < 0) {
                double scale = Math.max(0, Math.min(1,
                        (turretDegrees + 240.0) / 40.0));
                power *= scale;
            }

            turret.setPower(power);

            telemetry.addData("Turret encoder", turret.getCurrentPosition());
            telemetry.addData("Turret power", turret.getPower());
            telemetry.addData("Turret angle", "%.1f degrees", turretDegrees);




            if (0.1 <= gamepad1.left_trigger) {
                Z_Weight = 0.5;
                Speed = 0.25;
            } else if (0.1 <= gamepad1.right_trigger) {
                Z_Weight = .75;
                Speed = .75;
            } else {
                Speed = 1;
                Z_Weight = .75;
            }



            headingCurrent = -Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS));
            if (headingCurrent < 0) {
                headingCurrent = headingCurrent + 360;
            }

            //stick directions
            Y = -gamepad1.left_stick_x;
            X = gamepad1.left_stick_y;
            Z = -gamepad1.right_stick_x * Z_Weight;
            //driving math
            FLTargetPower = (Speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + Z));
            BLTargetPower = (Speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + Z));
            FRTargetPower = (Speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + -Z));
            BRTargetPower = (Speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -Z));

            FR.setPower(FRTargetPower);
            FL.setPower(FLTargetPower);
            BR.setPower(BRTargetPower);
            BL.setPower(BLTargetPower);

//            //LIMELIGHT MATH
//            LLResult result = LL.getLatestResult();
//            if (result != null) {
//                if (result.isValid()) {
//                    Pose3D botpose = result.getBotpose();
//                    double x = botpose.getPosition().x;
//                    double y = botpose.getPosition().y;
//                }
//            }

            //tune
            //LAUNCHER PIDS
            //double  = .5;
            double launchererror = targetlauncher - launcherV;
            double derlauncherThing = launchererror - lastlaunchererror;
            double launcheroutput = (launchererror * launcherP) + (derlauncherThing * launcherD);
            lastlaunchererror = launchererror;
            launcher.setVelocity(launcheroutput);
            telemetry.addData("Launcher Velo", launcher.getVelocity());
            targetlauncher = setlaunchervelo;
//            setlaunchervelo = 1600;




//button x
            if (gamepad1.x && txx) {
                sx = !sx;
                txx = false;
            }
            if (!gamepad1.x) {
                txx = true;
            }
            if (sx) {
                //open

            }
            if (!sx) {
                //close
            }

//button a
            if (gamepad1.dpad_down && ta) {
                sa = !sa;
                ta = false;
            }
            if (!gamepad1.dpad_down) {
                ta = true;
            }
            if (sa) {
                //one action
                //down
                ramp.setPosition(.5);

            }
            if (!sa) {
                //second action
                //up
                ramp.setPosition(.84);

            }

//button y
            if (gamepad1.a){
                stopper.setPosition(.97);
                intake.setPower(1);
            }
            if (gamepad1.dpad_up) {
                intake.setPower(0);
            }
//x launches
            if (gamepad1.x){
                stopper.setPosition(.81);
                intake.setPower(1);
                //targetlauncher = 1000000000;
            }

//button dpad down
            if (gamepad1.dpad_right && tdr) {
                sdr = !sdr;
                tdr = false;
            }
            if (!gamepad1.dpad_right) {
                tdr = true;
            }
            if (sdr) {
                //one action
            }
            if (!sdr) {
                //second action
            }
//button dpad left
            if (gamepad1.dpad_left && tdl) {
                sdl = !sdl;
                tdl = false;
            }
            if (!gamepad1.dpad_left) {
                tdl = true;
            }
            if (sdl) {
                //one action
            }
            if (!sdl) {
                //second action
            }
//button dpad up
            if (gamepad1.dpad_up && tdu) {
                sdu = !sdu;
                tdu = false;
            }
            if (!gamepad1.dpad_up) {
                tdu = true;
            }
            if (sdu) {
                //one action
            }
            if (!sdu) {
                //second action
            }
//button dpad down
            if (gamepad1.dpad_down && tdd) {
                sdd = !sdd;
                tdd = false;
            }
            if (!gamepad1.dpad_down) {
                tdd = true;
            }
            if (sdd) {
                //one action
            }
            if (!sdd) {
                //second action
            }


            //possible turret code
            telemetry.addData("Robot X inches",
                    pinpoint.getPosX(DistanceUnit.INCH));

            telemetry.addData("Robot Y inches",
                    pinpoint.getPosY(DistanceUnit.INCH));

            telemetry.addData("Robot heading degrees",
                    pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.update();
}}}