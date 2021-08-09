#include "galileo_parser.h"
#include <memory>

int main() {
  std::string file = "../../data/COM3_210730_115228.ubx";
  std::unique_ptr<GalileoParser> data = std::make_unique<GalileoParser>(file);
  data->Read();

  return 0;
}