#ifndef TELEOPHAND_FINGER_DATA_H
#define TELEOPHAND_FINGER_DATA_H

#include <cstdint>

namespace mc {

struct finger_data
{
  float distance_mm;
  float red_x, red_y, red_z;
  float blue_x, blue_y, blue_z;
};

static_assert(sizeof(finger_data) == 28, "finger_data must be 28 bytes for UDP transfer");

} // namespace mc
#endif //TELEOPHAND_FINGER_DATA_H
