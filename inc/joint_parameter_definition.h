namespace mc {
  enum type
  {
    command,
    response,
    reference,
    compensation,
    output,
    lowpass,
    type_size
  };

  enum state
  {
    pulse_count,
    x,
    dx,
    ddx,
    f,
    f_dis,
    f_vol,
    f_react,
    voltage,
    state_size
  };

  enum parameter
  {
    mass,
    damper,
    spring,
    k_p,
    k_v,
    k_f,
    mass_virtual,
    damper_virtual,
    spring_virtual,
    g_dis,
    g_diff,
    g_lpf,
    force_to_voltage,
    force_limit,
    pulse_per_rotation,
    multiplication,
    rotate_or_linear,
    gear_ratio,
    position_inverse,
    output_inverse,
    parameter_size
  };
} // namespace mc
