package frc.lib;

/**
 * An iterative boolean latch that delays the transition from false to true.
 *
 * @see <a
 *     href="https://github.com/frc1678/C2023-Public/blob/main/src/main/java/com/team1678/lib/util/DelayedBoolean.java">From
 *     1678</a>
 */
public class DelayedBoolean {

  private boolean previousValue;
  private double transitionTimestampSeconds;

  private final double delaySeconds;

  public DelayedBoolean(double initialTimestampSeconds, double delaySeconds) {
    previousValue = false;
    transitionTimestampSeconds = initialTimestampSeconds;
    this.delaySeconds = delaySeconds;
  }

  public boolean update(double timestampSeconds, boolean value) {
    boolean result = false;

    if (value && !previousValue) {
      transitionTimestampSeconds = timestampSeconds;
    }

    if (value && (timestampSeconds - transitionTimestampSeconds > delaySeconds)) {
      result = true;
    }

    previousValue = result;
    return result;
  }
}
