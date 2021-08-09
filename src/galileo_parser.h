#include <fstream>
#include <iostream>
#include <string>

class GalileoParser {
 private:
  std::string file_;
  std::ifstream raw_data_;

  uint8_t byte_;

  const uint8_t SYNC_HEADER_1_ = 0xb5;
  const uint8_t SYNC_HEADER_2_ = 0x62;
  
  bool sync_lock_1_ = false;
  bool sync_lock_2_ = false;

  struct MessageData{
    uint8_t message_class;
    uint8_t message_id;
    uint16_t length;
  } msg_data;

  struct NavigationData{
    uint8_t gnssId;
    uint8_t svId;
    uint8_t reserved0;
    uint8_t freqId;
    uint8_t numWords;
    uint8_t chn;
    uint8_t version;
    uint8_t reserved1;
  } payload;

  uint32_t words[9];

  int galileo_e1_num = 0;
  int galileo_e5_num = 0;
  int gps_num_ = 0;
  int sbas_num_ = 0;
  int beidou_num_ = 0;
  int qzss_num_ = 0;
  int glonass_num_ = 0;

 public:
  explicit GalileoParser(const std::string &path);
  
  void Read();
  void CheckSyncHeaders(uint8_t& byte_);
  bool ParseInitialData(std::ifstream& raw_data_);
  void ParsePayloadData(std::ifstream& raw_data_);
  void GnssCount(NavigationData& payload);
};
