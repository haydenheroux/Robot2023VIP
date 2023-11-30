package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Stack;

public class TripTracker {

  public static class Trip {
    public Pose2d start;
    public Pose2d end;
    public boolean hasEnded;
  }

  private final Stack<Trip> trips;

  public TripTracker() {
    this.trips = new Stack<Trip>();
  }

  public void start() {
    if (onTrip()) stop();

    Trip next = new Trip();

    next.start = Odometry.getInstance().getPose();

    trips.push(next);
  }

  public void stop() {
    if (onTrip() == false) return;

    Trip previous = trips.pop();

    previous.end = Odometry.getInstance().getPose();
    previous.hasEnded = true;

    trips.push(previous);
  }

  public boolean onTrip() {
    if (trips.isEmpty()) return false;

    return trips.peek().hasEnded == false;
  }

  public Transform2d getDistance() {
    if (trips.isEmpty()) return new Transform2d();

    if (onTrip()) {
      Pose2d start = trips.peek().start;
      Pose2d pose = Odometry.getInstance().getPose();

      return new Transform2d(start, pose);
    } else {
      Trip trip = trips.peek();

      return new Transform2d(trip.start, trip.end);
    }
  }
}
