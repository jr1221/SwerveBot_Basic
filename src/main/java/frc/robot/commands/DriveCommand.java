package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;

public class DriveCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private XboxController m_rXboxController;

    public DriveCommand(DriveSubsystem driveSubsystem, XboxController rXboxController) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_rXboxController = rXboxController;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(m_rXboxController.getY(GenericHID.Hand.kLeft), // ForWARD
                m_rXboxController.getX(GenericHID.Hand.kLeft), // SIDE
                m_rXboxController.getX(GenericHID.Hand.kRight) // TURN
        );
        // m_driveSubsystem.drive(0, 0, 0.001, false);
    }
}
