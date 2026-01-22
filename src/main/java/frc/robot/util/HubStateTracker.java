package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubStateTracker {

    private static String currentGameData = "";

    /**
     * Determines if the active hub matches the team's alliance.
     * 
     * @return true if the active hub is the same as the team's alliance, false otherwise or if game data is unavailable.
     */
    public static boolean isAllianceHubActive() {
        Alliance activeAlliance = getActiveAlliance();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (activeAlliance == null) {
            return false;
        }

        return activeAlliance == teamAlliance;
    }

    /**
     * Gets the currently active alliance based on match time and game data.
     * 
     * @return The active Alliance (Red or Blue), or null if game data is unavailable.
     */
    public static Alliance getActiveAlliance() {
        //Note: to test the logic in this implementation, see https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html#testing-game-specific-data
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        var currentTime = DriverStation.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        //See https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html for string format
        //Note: initialInactiveAlliance is the Alliance whose hub will be inactive in Shift 1
        Alliance initialInactiveAlliance = DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        // Tele-op total = 2:20 = 140 seconds
        // 140 - 130 = Transition shift period (2:20 - 2:10) -> hub is active for both alliances
        // 129 - 105 = Shift 1 (2:10 - 1:45) -> hub is active for alliance that scored LESS in Autonomous
        // 104 - 80 = Shift 2 (1:45 - 1:20) -> hub is active for alliance that scored MORE in Autonomous
        // 79 - 55 = Shift 3 (1:20 - 0:55) -> hub is active for alliance that scored LESS in Autonomous
        // 54 - 30 = Shift 4 (0:55 - 0:30) -> hub is active for alliance that scored MORE in Autonomous
        // 30 - 00 = End Game (0:30 - 0:00) -> hub is active for both alliances

        if (currentTime >= 130 || currentTime < 30) {
            //Transition Shift and End Game
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        } else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
            //Shift 1 and Shift 3
            return initialInactiveAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
        } else {
            //Shift 2 and Shift 4
            return initialInactiveAlliance;
        }
    }
}
