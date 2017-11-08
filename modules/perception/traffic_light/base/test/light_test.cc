#include <gtest/gtest.h>
#include <sstream>

#include "module/perception/traffic_light/base/light.h"

namespace adu {
namespace perception {
namespace traffic_light {

TEST(LightTest, test_light) {
  LightStatus light_status;
  {
    std::ostringstream oss;
    oss << "Status: [color:" << "unknown color" << " confidence:" << 0.0
        << " tracking_time:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = RED;
  {
    std::ostringstream oss;
    oss << "Status: [color:" << "red" << " confidence:" << 0.0
        << " tracking_time:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = GREEN;
  {
    std::ostringstream oss;
    oss << "Status: [color:" << "green" << " confidence:" << 0.0
        << " tracking_time:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = YELLOW;
  {
    std::ostringstream oss;
    oss << "Status: [color:" << "yellow" << " confidence:" << 0.0
        << " tracking_time:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = BLACK;
  {
    std::ostringstream oss;
    oss << "Status: [color:" << "black" << " confidence:" << 0.0
        << " tracking_time:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }
}

}
}
}