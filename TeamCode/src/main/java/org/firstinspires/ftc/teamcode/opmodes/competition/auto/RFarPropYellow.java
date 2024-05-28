package org.firstinspires.ftc.teamcode.opmodes.competition.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision;

@Autonomous(name="Red Far Prop 2+0")
public class RFarPropYellow extends CommandOpMode {
    private Mecanum2024 m_mecnaumDrive;
    private Lift m_lift;
    private Vision m_vision;
    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs();
        m_mecnaumDrive = new Mecanum2024(hardwareMap, configs, FieldPose2024.AutoRedFar);
        m_lift = new Lift(hardwareMap, "leftLift", "rightLift");
//        m_vision = new Vision();
    }
}
