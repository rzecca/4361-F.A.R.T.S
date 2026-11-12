package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
public class Kitchenet {



    //Drive Config
    private PIDController DriveController;
    private PIDController StrafeController;


    public static double pDrive;
    public static double iDrive;
    public static double dDrive;

    public static double pStrafe;
    public static double iStrafe;
    public static double dStrafe;

    public static double pHeading;
    public static double iHeading;
    public static double dHeading;
    public double drivepid;
    public double strafepid;
    public double headingpid;
    public double drivevelocity;
    public double strafevelocity;
    public double headingvelocity;

    public static double speed;
    public double DefaultSpeed;
    public static double Z_Weight;

    public static double xTarget;
    public static double yTarget;
    public static double PrexTarget;
    public static double PreyTarget;
    public static double headingTarget;


    double laserrorheading;
    double intThingheading;
    public double headingerror;
    double laserrordrive;
    double IntThingdrive;
    double driveerror;

    double laserrorstrafe;
    double IntThingstrafe;
    double strafeerror;


    public double xCurrent;
    public double yCurrent;
    public double headingCurrent;

    public double FrontRightTargetVelocity;
    public double FrontLeftTargetVelocity;
    public double BackRightTargetVelocity;
    public double BackLeftTargetVelocity;
    public double FrontRightTargetPower;
    public double FrontLeftTargetPower;
    public double BackRightTargetPower;
    public double BackLeftTargetPower;

    public Vector2d PureErrorVector = new Vector2d(0,0);
    public Vector2d PureTargetVector = new Vector2d(0,0);
    public Vector2d TargetPath;
    public Vector2d PastPath;
    public double TargetPathSqurd;
    public double ScalarProj;
    public Vector2d PointOnPath;
    public Vector2d ToPath;
    public Vector2d TrueError = new Vector2d(0,0);
    public double Strict;
    public double Eager;

    public ElapsedTime Timer;
    public int i = 1;

    public Pose2D startpose;

    public DcMotorEx FrontRight;
    public DcMotorEx FrontLeft;
    public DcMotorEx BackRight;
    public DcMotorEx BackLeft;


    GoBildaPinpointDriver odo;
    public double odoYawScalor;
    public double odoXoffSet;
    public double odoYoffSet;
    public int odoCalabrasionSleep;

    public VoltageSensor ControlHub_VoltageSensor;
    public VoltageSensor ExpansionHub2_VoltageSensor;



    private LinearOpMode myOpMode;

    public Kitchenet(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();






    public DcMotorEx intake;
    public DcMotorEx launcher;
    public Servo stopper;
    public Servo ramp;

    private double targetlauncher = 0;
    double launcherV;
    double lastlaunchererror;

    public void setLauncherSpeed(double ticksPerSecond) {
        targetlauncher = ticksPerSecond;
    }




    public void initializeAuto(){


        //Drive Init
        laserrorheading = 0;
        intThingheading = 0;
        laserrordrive = 0;
        IntThingdrive = 0;
        driveerror = 0;

        laserrorstrafe = 0;
        IntThingstrafe = 0;
        strafeerror = 0;

        pDrive = 70;
        iDrive = 0;
        dDrive = 5;
//60,4
        //strafe p should be a bit higher than drive p as it is hard and d should be a bit lowwer
        pStrafe = 75;
        iStrafe = 0;
        dStrafe = 5;

        pHeading = 17;
        iHeading = 0;
        dHeading = 1;

        odoYawScalor = 1;
        odoXoffSet = -6.5;
        odoYoffSet = 3;
        odoCalabrasionSleep = 500;

        Eager = 1;
        Strict = 1;

        DefaultSpeed = 1;


        FrontLeft = MotorMap("FL", DcMotorEx.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight = MotorMap("FR", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft = MotorMap( "BL", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight = MotorMap( "BR",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        DriveController = new PIDController(pDrive,iDrive,dDrive);
        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);

        odo.setOffsets(odoXoffSet, odoYoffSet, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //   odo.setYawScalar(odoYawScalor);

        odo.resetPosAndIMU();
        myOpMode.sleep(odoCalabrasionSleep);

        if (startpose != null) {
            odo.setPosition(startpose);
        } else {
            xTarget = 0;
            yTarget = 0;
            PrexTarget = 0;
            PreyTarget = 0;
            headingTarget = 0;
        }

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Timer = new ElapsedTime();


        //Personal Init

        intake = MotorMap( "intake", DcMotorEx.Direction.FORWARD , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher = MotorMap( "launcher", DcMotorEx.Direction.REVERSE , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopper = ServoSetup("stopper", Servo.Direction.FORWARD);
        ramp = ServoSetup("ramp", Servo.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void StartUpAuto(){


        //Drive Start Up
        Timer.reset();

        //Personal Start Up

    }


    public void initializeTele(){

        //Drive Init

        Z_Weight = 0.75;

        odoYawScalor = 1;
        odoXoffSet = -147.64;
        odoYoffSet = 63.50;
        odoCalabrasionSleep = 500;

        FrontLeft = MotorMap("FrontLeft", DcMotorEx.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight = MotorMap("FrontRight", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft = MotorMap( "BackLeft", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight = MotorMap( "BackRight",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(odoXoffSet, odoYoffSet);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.setYawScalar(odoYawScalor);

        odo.recalibrateIMU();
        myOpMode.sleep(odoCalabrasionSleep);

        odo.setPosition(startpose);

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();


        //Personal Init

        intake = MotorMap( "intake", DcMotorEx.Direction.FORWARD , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher = MotorMap( "launcher", DcMotorEx.Direction.REVERSE , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopper = ServoSetup("stopper", Servo.Direction.FORWARD);
        ramp = ServoSetup("ramp", Servo.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void DriveUpdateTele(double xstick, double ystick, double headingstick, double speed) {


        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
        yCurrent = -CurrentLocation.getY(DistanceUnit.INCH);
        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);

        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }

        double X = xstick;
        double Y = ystick;
        double Z = headingstick * Z_Weight;

        FrontLeftTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + Z));
        BackLeftTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + Z));
        FrontRightTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + -Z));
        BackRightTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -Z));

        FrontRight.setPower(FrontRightTargetPower);
        FrontLeft.setPower(FrontLeftTargetPower);
        BackRight.setPower(BackRightTargetPower);
        BackLeft.setPower(BackLeftTargetPower);


    }


    public void PersonalUpdateTele(){




    }



    public void DriveUpdateAuto() {
        double lastlaunchererror;
        double targetlauncher;
        double LAUNCHERVELO;
        double launchererror;
        double launcherV;

        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
        yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);

        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }

//        TargetPath = new Vector2d((xTarget - PrexTarget),(yTarget - PreyTarget));
//        PastPath = new Vector2d((xCurrent-PrexTarget),(yCurrent-PreyTarget));
//        TargetPathSqurd = (TargetPath.getX() * TargetPath.getX()) + (TargetPath.getY() * TargetPath.getY());
//        ScalarProj = PastPath.dot(TargetPath) / TargetPathSqurd;
//        PointOnPath = (new Vector2d(PrexTarget,PreyTarget)).plus(TargetPath.times(ScalarProj));
//        ToPath = PointOnPath.minus(new Vector2d(xCurrent,yCurrent));

        PureErrorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
        PureTargetVector = PureErrorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

//        TrueError = (ToPath.times(Strict)).plus(PureTargetVector.times(Eager));


//        drivepid = DrivePID(pDrive, iDrive, dDrive, TrueError.getX());
//        strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TrueError.getY());
        drivepid = DrivePID(pDrive, iDrive, dDrive, PureTargetVector.getX());
        strafepid = StrafePID(pStrafe, iStrafe, dStrafe, PureTargetVector.getY());
        headingpid = HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);
//if weird issue down line we reversed heading pid
        drivevelocity = drivepid;
        strafevelocity = strafepid;
        headingvelocity = headingpid;

        //hard code bad
        //speed = 1;

        FrontRightTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
        FrontLeftTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
        BackRightTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
        BackLeftTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

        FrontRight.setVelocity(FrontRightTargetVelocity);
        FrontLeft.setVelocity(FrontLeftTargetVelocity);
        BackRight.setVelocity(BackRightTargetVelocity);
        BackLeft.setVelocity(BackLeftTargetVelocity);
    }


    public void PersonalUpdateAuto() {
//        launcherV = (LLAUNCHER.getVelocity());
//        //tune
//        //LAUNCHER PIDS
//        // max velo is 2150
//
//        double launchererror = targetlauncher - launcherV;
//        double derlauncherThing = launchererror - lastlaunchererror;
//        double theithingthatdosesomething =+ launchererror;
//        double launcheroutput = (launchererror * launcherP) + (derlauncherThing * launcherd) + (theithingthatdosesomething * launcheri) + (LLAUNCHER.getVelocity());
//        lastlaunchererror = launchererror;
//        LLAUNCHER.setVelocity(launcheroutput);
//        RLAUNCHER.setVelocity(launcheroutput);

        //tune
        //LAUNCHER PIDS
        double launcherP = .3;
        double launcherD = .002;

//        double launchererror = targetlauncher - launcherV;
//        double derlauncherThing = launchererror - lastlaunchererror;
//        double launcheroutput = (launchererror * launcherP) + (derlauncherThing * launcherD);
//        lastlaunchererror = launchererror;
//        launcher.setVelocity(launcheroutput);


            launcherV = launcher.getVelocity();

            if (targetlauncher == 0) {
                launcher.setPower(0);
                lastlaunchererror = 0;
                return;
            }


            double launchererror = targetlauncher - launcherV;
            double derlauncherThing = launchererror - lastlaunchererror;
            double launcheroutput =
                    launchererror * launcherP
                            + derlauncherThing * launcherD;

            lastlaunchererror = launchererror;
            launcher.setVelocity(launcheroutput);

    }

    DcMotorEx MotorMap(String deviceName, DcMotorEx.Direction direction, DcMotor.RunMode EncoderMode) {
        DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        aMotor.setMode(EncoderMode);
        return aMotor;
    }


    Servo ServoSetup(String deviceName, Servo.Direction direction) {
        Servo aServo = myOpMode.hardwareMap.get(Servo.class, deviceName);
        aServo.setDirection(direction);
        return aServo;
    }


    public double HeadingPID(double p, double i, double d, double current, double target){

        headingerror = target - current;

        if (headingerror > 180.0){
            headingerror = 360 - headingerror;
        } else if (headingerror < -180.0) {
            headingerror = 360 + headingerror;
        }

        intThingheading += headingerror;
        double derThing = headingerror - laserrorheading;
        double output = (headingerror * p) + (intThingheading * i) + (derThing * d);
        laserrorheading = headingerror;

        return output;
    }

    public double DrivePID(double p, double i, double d, double error){

        driveerror = error;
        IntThingdrive += driveerror;
        double derThing = driveerror - laserrordrive;
        double output = (driveerror * p) + (IntThingdrive * i) + (derThing * d);
        laserrordrive = driveerror;

        return output;
    }

    public double StrafePID(double p, double i, double d, double error){

        strafeerror = error;
        IntThingstrafe += strafeerror;
        double derThing = strafeerror - laserrorstrafe;
        double output = (strafeerror * p) + (IntThingstrafe * i) + (derThing * d);
        laserrorstrafe = strafeerror;

        return output;
    }


    public void telemetryupdate(){


        //Drive Telem
        myOpMode.telemetry.addLine("FARTS Data");

        myOpMode.telemetry.addData("Xerror ", PureErrorVector.getX());
        myOpMode.telemetry.addData("Yerror ", PureErrorVector.getY());
        myOpMode.telemetry.addData("headingerror ",headingerror);
        myOpMode.telemetry.addData("XTarget Vec ", PureTargetVector.getX());
        myOpMode.telemetry.addData("YTarget Vec ", PureTargetVector.getY());
        myOpMode.telemetry.addData("YTarget Vec ", PureTargetVector.getY());

        myOpMode.telemetry.addData("drive volicirty ", drivevelocity);
        myOpMode.telemetry.addData("strafte velocity ", strafevelocity);
        myOpMode.telemetry.addData("Heading Vec ", headingvelocity);



        myOpMode.telemetry.addData("FrontRight ", FrontRight.getVelocity());
        myOpMode.telemetry.addData("FrontLeft ", FrontLeft.getVelocity());
        myOpMode.telemetry.addData("BackRight ", BackRight.getVelocity());
        myOpMode.telemetry.addData("BackLeft ", BackLeft.getVelocity());

        myOpMode.telemetry.addData("FrontRight Target ", FrontRightTargetVelocity);
        myOpMode.telemetry.addData("FrontLeft Target ", FrontLeftTargetVelocity);
        myOpMode.telemetry.addData("BackRight Target ", BackRightTargetVelocity);
        myOpMode.telemetry.addData("BackLeft Target ", BackLeftTargetVelocity);

        myOpMode.telemetry.addData("Target X ",xTarget);
        myOpMode.telemetry.addData("Target Y ",yTarget);
        myOpMode.telemetry.addData("Target Heading",headingTarget);

        myOpMode.telemetry.addData("Current X ",xCurrent);
        myOpMode.telemetry.addData("Current Y ",yCurrent);
        myOpMode.telemetry.addData("Current Heading ",headingCurrent);
        myOpMode.telemetry.addData("Speed ",speed);

        myOpMode.telemetry.addData("Battery Voltage", ControlHub_VoltageSensor.getVoltage());

        myOpMode.telemetry.addData("yaw scalor", odo.getYawScalar());

        myOpMode.telemetry.addData("i", i);


        TelemetryPacket packet1 = new TelemetryPacket();
        packet1.fieldOverlay()
                .drawGrid(0, 0, 144, 144, 7, 7)
                .setStroke("goldenrod")
                .strokeCircle(xCurrent, yCurrent, 4)
                .setStroke("blue")
                .strokeCircle(xTarget, yTarget, 4);


        //Personal Telem




        dashboard.sendTelemetryPacket(packet1);
        myOpMode.telemetry.update();

    }



    public void startlocation(double x, double y, double heading){
        double rotation;
        if (heading < 180){
            rotation = -heading;
        } else {
            rotation = -(heading - 360);
        }
        startpose = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES, rotation);

        xTarget = startpose.getX(DistanceUnit.INCH);
        yTarget = startpose.getY(DistanceUnit.INCH);
        PrexTarget = startpose.getX(DistanceUnit.INCH);
        PreyTarget = startpose.getY(DistanceUnit.INCH);
        headingTarget = startpose.getHeading(AngleUnit.DEGREES);

    }


    public void ResetPinpoint (){
        myOpMode.sleep(odoCalabrasionSleep);
        //odo.setYawScalar(odoYawScalor);
        odo.recalibrateIMU();
        myOpMode.sleep(odoCalabrasionSleep);

    }

    public void iaamrunning (){

        myOpMode.telemetry.addData("hellps", i);

    }

    public void Counter(double Time){
        if (Timer.seconds() > Time) {
            i += 1;
            Timer.reset();
        }
    }



    public void driveTo(double x, double y) {

//            if (x!=xTarget || y!=yTarget){
//                PrexTarget=xTarget;
//                PreyTarget=yTarget;
//            }

        xTarget = x;
        yTarget = y;
        speed = DefaultSpeed;

    }

    public void driveTo(double x, double y, double heading) {

//            if (x!=xTarget || y!=yTarget){
//                PrexTarget=xTarget;
//                PreyTarget=yTarget;
//            }

        xTarget = x;
        yTarget = y;
        headingTarget = heading;
        speed = DefaultSpeed;

    }
    //how to add actions for auto
    // public void action.name(doublevalue){
    // action; }
    public void intakereset(double pp){
        intake.setPower(pp);
    }


    public void driveTo(double x, double y, double heading, double Speed) {

//            if (x!=xTarget || y!=yTarget){
//                PrexTarget=xTarget;
//                PreyTarget=yTarget;
//            }

        xTarget = x;
        yTarget = y;
        headingTarget = heading;
        speed = Speed;
    }


}