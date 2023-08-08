#pragma once

#include "oh_my_loam/extractor/extractor.h"

namespace oh_my_loam {

// for VLP-general
class ExtractorVLP : public Extractor {
 public:
  ExtractorVLP() {
    num_scans_ = 16;
  }

 private:
  int GetScanID(const common::Point &pt) const override;
};

}  // namespace oh_my_loam