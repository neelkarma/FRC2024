package frc.robot.utils.RangeMath;

import java.util.Optional;

public class RangeSettings {
  protected double minThrottle = 0; // >= 0
  protected double maxThrottle = 1; // <= 1
  protected double minTurn = 0;     // >= 0
  protected double maxTurn = 1;     // <= 1

  protected double turnCurvePower = 1;     // often 2 to 4
  protected double throttleCurvePower = 1;     // often 2 to 4
  
  protected double throttleEffectOnTurn = 1;


  public RangeSettings(){

  }

  /**
   * 
   * @param minThrottle
   * @param maxThrottle
   * @param minTurn
   * @param maxTurn
   * @param throttleEffect
   * @return
   */
  public static RangeSettings InitTankBot(double minThrottle, double maxThrottle, double throttleCurve,
                                      double minTurn, double maxTurn, double turnCurve, double throttleEffect){
    RangeSettings rangSets = new RangeSettings();
    rangSets.setThrottle(minThrottle, maxThrottle, throttleCurve);
    rangSets.setTurn(minTurn, maxTurn, turnCurve, throttleEffect);
    return rangSets;
  }

  /**
   * 
   * @param minThrottle
   * @param maxThrottle
   * @param minTurn
   * @param maxTurn
   * @param throttleEffect
   * @return
   */
  public static RangeSettings InitSwerveBot(double minThrottle, double maxThrottle, double throttleCurve,
                                      double minTurn, double maxTurn, double turnCurve){
    RangeSettings rangSets = new RangeSettings();
    rangSets.setThrottle(minThrottle, maxThrottle, throttleCurve);
    rangSets.setTurn(minTurn, maxTurn, turnCurve, 0);
    return rangSets;
  }

  /**
   * 
   * @param ThrottleRange [min, max]
   * @param Throttlecurve
   */
  public void setThrottle(double minThrottle, double maxThrottle, double throttleCurve){
    if(minThrottle > 1 || maxThrottle < 0 || minThrottle > maxThrottle){
      throw new IllegalArgumentException("Min > 1, Max < 0, or Min > Max");
    }
    this.minThrottle = minThrottle;
    this.maxThrottle = maxThrottle;
    this.throttleCurvePower = throttleCurve;
  }

  /**
   * 
   * @param ThrottleRange [min, max]
   * @param Throttlecurve
   */
  public void setTurn(double minTurn, double maxTurn, double turnCurve, double throttleEffect){
    if(minTurn > 1 || maxTurn < 0 || minTurn > maxTurn){
      throw new IllegalArgumentException("Min > 1, Max < 0, or Min > Max");
    } else if(throttleEffect < 0 || throttleEffect > 1) {
      throw new IllegalArgumentException("throttleEffect < 0 or > 1");
    }
    this.minTurn = minTurn;
    this.maxTurn = maxTurn;
    this.throttleCurvePower = turnCurve;
    this.throttleEffectOnTurn = throttleEffect;
  }
}
