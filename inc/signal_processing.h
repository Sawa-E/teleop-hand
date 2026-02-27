#ifndef MOTIONCONTROL_SIGNAL_PROCESSING_H
#define MOTIONCONTROL_SIGNAL_PROCESSING_H
#include <iostream>

namespace mc {
  template<typename T>
  class signal
  {
  public:
    static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
    {
      buf += dt * (input - output);
      output = cutoff_frequency * (buf);
    }

    static void high_pass_filter(T dt,  T cutoff_frequency, T &output, T &input, T &buf)
    {
      // TODO
      std::cout << "High pass fileter is not implemented" << std::endl;
    }

    static void band_pass_filter(T dt,  T cutoff_frequency, T &output, T &input, T &buf)
    {
      // TODO
      std::cout << "Band pass fileter is not implemented" << std::endl;
    }

    static void pseudo_differential(T dt,  T cutoff_frequency, T &output, T &input, T &buf)
    {
      buf += dt * output;
      output = cutoff_frequency * (input - buf);
    }

    static void disturbance_observer(T dt, T dob_gain, T &f_dis, T &f_out, T &dx, T &buf, T M, T D=0, T K=0)
    {
      (void)(D);
      (void)(K);
      f_dis = buf - dob_gain * M * dx;
      buf += dt * dob_gain *(f_out + M * dob_gain * dx - buf);
    }
  };
} // namespace mc
#endif //MOTIONCONTROL_SIGNAL_PROCESSING_H
