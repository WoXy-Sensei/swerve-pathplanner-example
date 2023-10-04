package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.commands.PrintCommand;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
public class test extends SequentialCommandGroup {
    public test(Swerve m_Swerve) {
    
        PathPlannerTrajectory trajectory;

  
        trajectory = PathPlanner.loadPath("test", PathPlannerConstants.constraints);
        HashMap<String,Command> events = new HashMap<>();
        events.put("event1", new PrintCommand("test event"));
        addCommands(m_Swerve.getAutoBuilder(events).fullAuto(trajectory));
       
    }
}
