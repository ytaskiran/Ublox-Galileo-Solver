#include "galileo_parser.h"

GalileoParser::GalileoParser(const std::string& path) : file_(path) {}

void GalileoParser::Read() {
  raw_data_.open(file_, std::ios::binary);

  if (!raw_data_.is_open()) {
    std::cout << "File cannot be read" << std::endl;
  }

  int counter = 0;

  while (!raw_data_.eof()) {
    raw_data_.read(reinterpret_cast<char*>(&byte_), sizeof(byte_));

    CheckSyncHeaders(byte_);

    if (sync_lock_1_ && sync_lock_2_) {
      bool msg_type = ParseInitialData(raw_data_);

      if (msg_type){
        ParsePayloadData(raw_data_);
        if (payload.numWords == 9){
            raw_data_.read(reinterpret_cast<char*>(&words), sizeof(words));
            std::cout << msg_data.length << std::endl;
            break;
        }
      }

      sync_lock_1_ = false;
      sync_lock_2_ = false;
    }
  }
  std::cout << "\nGalileo E1: " << galileo_e1_num 
            << "\nGalileo E5: " << galileo_e5_num
            << "\nGPS: " << gps_num_
            << "\nGLONASS: " << glonass_num_
            << "\nBeidou: " << beidou_num_
            << "\nQZSS: " << qzss_num_
            << "\nSBAS: " << sbas_num_ 
            << std::endl;
}

void GalileoParser::CheckSyncHeaders(uint8_t& byte_) {
  if (!sync_lock_1_) {
    if (byte_ == SYNC_HEADER_1_) {
      sync_lock_1_ = true;
    }
  } else {
    if (byte_ == SYNC_HEADER_2_) {
      sync_lock_2_ = true;
    } else
      sync_lock_1_ = false;
  }
}

/*
Belki aşağıdaki fonksiyondan length verisi alınarak
eğer istenilen message type değilse hızlı ilerlemek için 
bir byte bir byte ilerlemesi yerine hızlı bir şekilde
seekg ile falan ilerletilebilir
*/

bool GalileoParser::ParseInitialData(std::ifstream& raw_data_) {
  raw_data_.read(reinterpret_cast<char*>(&msg_data),
                 sizeof(msg_data));

  if (msg_data.message_class == 0x02 && msg_data.message_id == 0x13){
    return true;
  }
  return false;
}

void GalileoParser::ParsePayloadData(std::ifstream& raw_data_) {
  raw_data_.read(reinterpret_cast<char*>(&payload), sizeof(payload));

  GnssCount(payload);
}

void GalileoParser::GnssCount(NavigationData& payload){
  switch (payload.gnssId){
    case 0:
      gps_num_++;
      break;
    case 1:
      sbas_num_++;
      break;
    case 2:
      if(payload.numWords == 8) galileo_e1_num++; // galileo total 23504
      else if(payload.numWords == 9) galileo_e5_num++;
      break;
    case 3:
      beidou_num_++;
      break;
    case 5:
      qzss_num_++;
      break;
    case 6:
      glonass_num_++;
      break;
  }
}