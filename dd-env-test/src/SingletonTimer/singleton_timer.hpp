//
// Created by daniel-bw on 12/13/23.
//

#ifndef LLC_NEW_INCLUDE_SINGLETONTIMER_SINGLETON_TIMER_HPP_
#define LLC_NEW_INCLUDE_SINGLETONTIMER_SINGLETON_TIMER_HPP_

#include <Arduino.h>

class SingletonTimer
{
public:
  static SingletonTimer& getInstance() {
    static SingletonTimer instance;
    return instance;
  }
  
  // Get the current timer count using millis()
  unsigned long getCurrentTime() const {
    return micros() / 1000;
  }

private:
  SingletonTimer() {} // Private constructor to enforce singleton pattern
  SingletonTimer(const SingletonTimer&) = delete; // Disable copy constructor
  void operator=(const SingletonTimer&) = delete; // Disable assignment operator
};

class TimerHelper
{
public:
  static bool timeHasPassed(unsigned long main_timer,
                            unsigned long &last_timestamp,
                            int spin_rate_in_ms,
                            float duty_cycle = 0.0)
  {
    if (abs(main_timer - last_timestamp) > spin_rate_in_ms*(1-duty_cycle))
    {
      if (abs(main_timer - last_timestamp) > spin_rate_in_ms)
      {
        last_timestamp = main_timer;
      }
      return true;
    }
    return false;
    
  }
  
  static unsigned long timeDelay(unsigned long main_timer, unsigned long delay_in_ms)
  {
    return main_timer - delay_in_ms;
  }
};

#endif //LLC_NEW_INCLUDE_SINGLETONTIMER_SINGLETON_TIMER_HPP_
