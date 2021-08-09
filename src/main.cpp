#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

struct NavigationData {
  uint8_t gnssId;
  uint8_t svId;
  uint8_t reserved0;
  uint8_t freqId;
  uint8_t numWords;
  uint8_t chn;
  uint8_t version;
  uint8_t reserved1;
};

int main() {
  char buffer[10000];

  std::ifstream data("../COM3_210730_115228.ubx", std::ios::binary);

  if (!data) {
    std::cout << "File cannot be read" << std::endl;
  }

  data.read(reinterpret_cast<char *>(&buffer), sizeof(buffer));
  data.read(reinterpret_cast<char *>(&buffer), sizeof(buffer));

  for (int i = 0; i < 10000; i++) {
    std::cout << buffer[i];
    if ((uint8_t)buffer[i] == 0xb5) {
      std::cout << "\n\n"
                << std::dec << (uint8_t)buffer[i] << "\n\nFOUND HEADER\n"
                << std::endl;
      return 0;
    }
  }

  data.close();

  if (!data.good())
    std::cout << "sth wtfd" << std::endl;

  return 0;
}
