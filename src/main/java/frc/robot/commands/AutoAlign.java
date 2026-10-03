package frc.robot.commands;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    public Command m_path;
    private boolean end = false;

    @Override
    public void initialize(){

        Pose2d endPose= new Pose2d(-4.25,-2.0,Rotation2d.fromDegrees(80.85));

        PathConstraints constraints = new PathConstraints(4, 2, 2 * Math.PI, 4 * Math.PI);

        final Command path = AutoBuilder.pathfindToPose(endPose, constraints, 0);
        m_path = path;
    }

    @Override
    public void execute(){

        m_path.schedule();
        end = true;
        
    }
    @Override
    public boolean isFinished(){
        return end;
    }
}
