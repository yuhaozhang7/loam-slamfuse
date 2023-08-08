#include "extractor_VLP.h"

#include "common/math/math_utils.h"
#include "common/registerer/registerer.h"

namespace oh_my_loam {

int ExtractorVLP::GetScanID(const common::Point &pt) const {
  double theta =
      common::Rad2Degree(std::atan2(pt.z, std::hypot(pt.x, pt.y))) + 15.0;
  return static_cast<int>(std::round(theta / 2.0) + 1.e-5);
};

REGISTER_CLASS(Extractor, ExtractorVLP)

}  // namespace oh_my_loam