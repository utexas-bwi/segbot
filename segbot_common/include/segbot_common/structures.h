#ifndef STRUCTURES_9TKSKPHM
#define STRUCTURES_9TKSKPHM

#include <opencv/cv.h>
#include <stdint.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace bwi_common {

  typedef cv::Point2f Point2f;

  void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t>& location_map);

  class Door {
    public:
      std::string name;
      std::string approach_names[2];
      Point2f approach_points[2];
      float approach_yaw[2];
      float width;
  };

  void readDoorFile(const std::string& filename, std::vector<Door>& doors);

} /* bwi_common */

#endif /* end of include guard: STRUCTURES_9TKSKPHM */
