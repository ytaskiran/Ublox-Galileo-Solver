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

  struct NavigationDataHead{
    uint8_t gnssId;
    uint8_t svId;
    uint8_t reserved0;
    uint8_t freqId;
    uint8_t numWords;
    uint8_t chn;
    uint8_t version;
    uint8_t reserved1;
  } payload_sfrbx_head;

  struct SignalInformationHead{
    uint32_t iTOW;
    uint8_t version;
    uint8_t numSigs;
    uint16_t reserved0;
  } payload_navsig_head;

  struct SignalInformation{
    uint8_t gnssId;
    uint8_t svId;
    uint8_t sigId;
    uint8_t freqId;
    int16_t prRes;
    uint8_t cno;
    uint8_t qualityInd;
    uint8_t corrSource;
    uint8_t ionoModel;
    unsigned int sigFlags : 16;
    uint32_t reserved1;
  } payload_navsig;

  enum MessageType {
    UBX_RXM_SFRBX,
    UBX_NAV_SIG,
    NOT_DEFINED
  } msg_type;

  uint32_t words[9];

  unsigned int galileo_num_sfrbx_ = 0;
  unsigned int gps_num_sfrbx_ = 0;
  unsigned int sbas_num_sfrbx_ = 0;
  unsigned int beidou_num_sfrbx_ = 0;
  unsigned int qzss_num_sfrbx_ = 0;
  unsigned int glonass_num_sfrbx_ = 0;

  unsigned int galileo_num_navsig_ = 0;
  unsigned int gps_num_navsig_ = 0;
  unsigned int sbas_num_navsig_ = 0;
  unsigned int beidou_num_navsig_ = 0;
  unsigned int qzss_num_navsig_ = 0;
  unsigned int glonass_num_navsig_ = 0;

  unsigned int rxm_sfrbx_counter = 0;
  unsigned int nav_sig_counter = 0;


 public:
  explicit GalileoParser(const std::string &path);
  
  void Read();
  void CheckSyncHeaders(uint8_t& byte_);
  void ParseInitialData(std::ifstream& raw_data_, MessageType& msg_type);
  void ParsePayloadData(std::ifstream& raw_data_);
  void GnssCount(NavigationDataHead& payload);
  void GnssCount(SignalInformation& payload);
  void Log();
  void Warn();
};
