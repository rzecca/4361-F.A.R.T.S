package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="CORRDGETTERFORTUNINGWITHFTCBIOBUZZWITHFARTSFULLYAUTONOMOUSROBOTICTRAJECTORYSYSTEM")
public class CORRDGETTERFORTUNINGWITHFTCBIOBUZZWITHFARTSFULLYAUTONOMOUSROBOTICTRAJECTORYSYSTEM extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i;


    @Override public void runOpMode() {


        stuff.initializeAuto();
        stuff.telemetryupdate();


        waitForStart();

        stuff.StartUpAuto();
        stuff.telemetryupdate();


        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.telemetryupdate();


            if (i == 1) {
                    stuff.driveTo(0,0);
            stuff.Counter(2);
            }


            else if (i == 2) {
                    break;
            }

        }
    }
}