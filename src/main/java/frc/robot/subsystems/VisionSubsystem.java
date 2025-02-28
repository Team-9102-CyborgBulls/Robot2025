package frc.robot.subsystems; // Déclaration du package où se trouve la classe VisionSubsystem

import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera; // Import des classes nécessaires
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase { // Déclaration de la classe VisionSubsystem qui étend la classe SubsystemBase

    // Constantes telles que la hauteur de la caméra et de la cible stockées. À changer selon le robot et l'objectif !
    public static final double CAMERA_HEIGHT_METERS = 0.032;
    public static final double TARGET_HEIGHT_METERS = 0.017;
    
    // Angle entre l'horizontal et la caméra.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    
    // Distance souhaitée par rapport à la cible
    public static final double GOAL_RANGE_METERS = 1;
    
    // Changer ceci pour correspondre au nom de votre caméra
   
    RobotContainer m_robotContainer;
    //objet getlatestresult
    //PhotonPipelineResult result = m_robotContainer.camera.getLatestResult();
    
    // Les constantes PID doivent être ajustées par robot
    final static double P_GAIN = 0.1;
    final static double D_GAIN = 0.0;
    public static PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

    public VisionSubsystem() {} // Constructeur de la classe VisionSubsystem

    @Override
    public void periodic() {
        
    } // Cette méthode sera appelée une fois par exécution du planificateur

}