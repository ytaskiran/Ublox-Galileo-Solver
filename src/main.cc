#include "galileo_solver.h"
#include <memory>

int main() {
  std::string file = "../../data/COM3_210730_115228.ubx";
  std::unique_ptr<GalileoSolver> data = std::make_unique<GalileoSolver>(file);
  data->read();

  return 0;
}