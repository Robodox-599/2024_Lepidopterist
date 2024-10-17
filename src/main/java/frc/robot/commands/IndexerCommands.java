package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands extends Command {
  public static Command stopIndexer(Indexer indexer) {
    return indexer.setSpeed(0);
  }

  public static Command runIndexer(Indexer indexer, double speed) {
    return indexer.setSpeed(speed);
  }
}
