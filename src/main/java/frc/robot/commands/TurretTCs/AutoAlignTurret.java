package frc.robot.commands.TurretTCs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import java.util.Optional;

public class AutoAlignTurret extends Command 
{
    private final Turret turret;
    private final NetworkTable limelightTable;
    private final PIDController pidController;

    public AutoAlignTurret(Turret turret) 
    {
        this.turret = turret;
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // idk the pids so something something goes here
        this.pidController = new PIDController(0.0, 0.0, 0.0); 
        
        addRequirements(turret);
    }

    @Override
    public void execute() 
  {
        double tv = limelightTable.getEntry("tv").getDouble(0);
        long tid = limelightTable.getEntry("tid").getInteger(-1);
        
        // does the limelight spy with its little eye something that starts with the letter AprilTag?
        if (tv == 1.0 && isTargetValidForAlliance(tid)) 
        {
            double tx = limelightTable.getEntry("tx").getDouble(0);
            
            // actual turret ,a
            double steeringAdjust = pidController.calculate(tx, 0.0);
            
            // change this based off our turret velocities
            steeringAdjust = Math.max(-0.4, Math.min(0.4, steeringAdjust));
            
            turret.setPower(steeringAdjust);
        } else 
        {
            // this was an issue we had a lot in ftc, the turret would just start spinning out of control, so this should stop it if we ever lose sight of the tags
            turret.setPower(0.0); 
        }
    }

    private boolean isTargetValidForAlliance(long tid) 
  {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) 
        {
            if (alliance.get() == Alliance.Red) 
            {
                return tid == 9 || tid == 10;
            } 
            else if (alliance.get() == Alliance.Blue) 
            {
                return tid == 25 || tid == 26;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.setPower(0.0);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
