#include "galileo_parser.h"
#include <bitset>

GalileoParser::GalileoParser(const std::string &path) : file_(path) {}

void GalileoParser::Read() {
  raw_data_.open(file_, std::ios::binary);

  if (!raw_data_.is_open()) {
    std::cout << "File cannot be read" << std::endl;
  }

  while (!raw_data_.eof()) {
    raw_data_.read(reinterpret_cast<char *>(&byte_), sizeof(byte_));

    CheckSyncHeaders(byte_);

    if (sync_lock_1_ && sync_lock_2_) {
      ParseInitialData(raw_data_, msg_type_);
      ParsePayloadData(raw_data_);
      pos_ = 0;

      sync_lock_1_ = false;
      sync_lock_2_ = false;
    }
  }
  Log();
  raw_data_.close();
}

void GalileoParser::CheckSyncHeaders(uint8_t &byte_) {
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

void GalileoParser::ParseInitialData(std::ifstream &raw_data_,
                                     MessageType &msg_type_) {
  raw_data_.read(reinterpret_cast<char *>(&msg_data), sizeof(msg_data));

  if (msg_data.message_class == 0x02 && msg_data.message_id == 0x13) {
    msg_type_ = UBX_RXM_SFRBX;
    rxm_sfrbx_counter++;
  } else if (msg_data.message_class == 0x01 && msg_data.message_id == 0x43) {
    msg_type_ = UBX_NAV_SIG;
    nav_sig_counter++;
  } else
    msg_type_ = NOT_DEFINED;
}

bool GalileoParser::ParsePayloadData(std::ifstream &raw_data_) {
  if (msg_type_ == UBX_RXM_SFRBX) {
    raw_data_.read(reinterpret_cast<char *>(&payload_sfrbx_head),
                   sizeof(payload_sfrbx_head));

    GnssCount(payload_sfrbx_head);

    if (payload_sfrbx_head.gnssId != 2)
      return false;

    /*raw_data_.read(reinterpret_cast<char *>(&payload_data_word_head),
                   sizeof(payload_data_word_head));

    std::cout << std::bitset<8>{payload_data_word_head.word_type} << '\n';*/

    unsigned int *dword = ReadWordToBuffer<unsigned int>();

    payload_data_word_head.even_odd = GetBits(*dword, 1);
    payload_data_word_head.page_type = GetBits(*dword, 1); 
    payload_data_word_head.word_type = GetBits(*dword, 6);

    if (payload_data_word_head.page_type == 1) // Skip alert pages
      return false;

    even_ = payload_data_word_head.even_odd;

    counter++;

    bool check_word_type = DetermineWordType(payload_data_word_head);

    if (!check_word_type)
      return false;

    true_counter++;

    ParseDataWord(raw_data_, dword);

    return true;
  }

  else if (msg_type_ == UBX_NAV_SIG) {
    raw_data_.read(reinterpret_cast<char *>(&payload_navsig_head),
                   sizeof(payload_navsig_head));

    for (int i = 0; i < payload_navsig_head.numSigs; i++) {
      raw_data_.read(reinterpret_cast<char *>(&payload_navsig),
                     sizeof(payload_navsig));
      GnssCount(payload_navsig);
    }
    return true;
  }
  return false;
}

bool GalileoParser::DetermineWordType(
    NavigationDataWordHead &payload_data_word_head) {
  switch (payload_data_word_head.word_type) {
  case 0:
    word_type_ = SPARE;
    return true;

  case 1:
    word_type_ = EPHEMERIS_1;
    return true;

  case 2:
    word_type_ = EPHEMERIS_2;
    return true;

  case 3:
    word_type_ = EPHEMERIS_3;
    return true;

  case 4:
    word_type_ = EPHEMERIS_4__CLOCK_CORRECTION;
    return true;

  case 5:
    word_type_ = IONOSPHERIC_CORRECTION__BGD__SIG_HEALTH__DVS__GST;
    return true;

  case 6:
    word_type_ = GST_UTC_CONVERSION;
    return true;

  case 7:
    word_type_ = ALMANAC_1;
    return true;

  case 8:
    word_type_ = ALMANAC_2;
    return true;

  case 9:
    word_type_ = ALMANAC_3;
    return true;

  case 10:
    word_type_ = ALMANAC_4;
    return true;

  case 16:
    word_type_ = REDUCED_CED;
    return true;

  case 17:
  case 18:
  case 19:
  case 20:
    word_type_ = FEC2;
    return true;

  case 63:
    word_type_ = DUMMY;
    return true;

  default:
    // Warn();
    false_counter++;
    return false;
  }
}

void GalileoParser::ParseDataWord(std::ifstream &raw_data_, unsigned int *dword_1) {
  if (word_type_ == EPHEMERIS_1){
    word_type_1.issue_of_data = GetBits(*dword_1, 10);
    word_type_1.reference_time = GetBits(*dword_1, 14);

    signed int *dword_2 = ReadWordToBuffer<signed int>();
    word_type_1.mean_anomaly = GetBits(*dword_2, 32);

    unsigned int *dword_3 = ReadWordToBuffer<unsigned int>();
    word_type_1.eccentricity = GetBits(*dword_3, 32);

    unsigned long long *dword_util = ReadWordUtilMiddle<unsigned long long>();
    word_util.tail = GetBits(*dword_util, 6);
    word_util.even_odd = GetBits(*dword_util, 1);
    word_util.page_type = GetBits(*dword_util, 1);

    if (word_util.tail != 0) Warn();

    if (even_ == 0) { if (word_util.even_odd != 1) Warn(); }
    if (even_ == 1) { if (word_util.even_odd != 0) Warn(); }


    unsigned long long *dword_data = ReadWordDataMiddle<unsigned long long>();
    word_type_1.root_semi_major_axis = GetBits(*dword_data, 32);
    word_type_1.reserved = GetBits(*dword_data, 2);
  }
}

template <typename T>
T* GalileoParser::ReadWordToBuffer(){
  pos_ = 0;
  raw_data_.read(reinterpret_cast<char *>(&dword_buffer_), sizeof(dword_buffer_));
  T *dword = reinterpret_cast<T *>(dword_buffer_);
  
  return dword;
}

template <typename T>
T* GalileoParser::ReadWordUtilMiddle(){
  pos_ = 0;
  raw_data_.read(reinterpret_cast<char *>(&big_dword_buffer_), sizeof(big_dword_buffer_));
  T *dword_util = reinterpret_cast<T *>(big_dword_buffer_);

  *dword_util = *dword_util & mask1_;
  *dword_util = (*dword_util >> 18) | (*dword_util >> 24);

  return dword_util;
}

template <typename T>
T* GalileoParser::ReadWordDataMiddle(){
  pos_ = 0;
  T *dword_data = reinterpret_cast<T *>(big_dword_buffer_);

  *dword_data = *dword_data & mask2_;
  *dword_data = (*dword_data) | (*dword_data >> 16);

  return dword_data;
}

void GalileoParser::GnssCount(NavigationDataHead &payload) {
  switch (payload.gnssId) {

  case 0:
    if (msg_type_ == UBX_RXM_SFRBX)
      gps_num_sfrbx_++;
    else
      Warn();
    break;

  case 1:
    if (msg_type_ == UBX_RXM_SFRBX)
      sbas_num_sfrbx_++;
    else
      Warn();
    break;

  case 2:
    if (msg_type_ == UBX_RXM_SFRBX)
      galileo_num_sfrbx_++;
    else
      Warn();
    break;

  case 3:
    if (msg_type_ == UBX_RXM_SFRBX)
      beidou_num_sfrbx_++;
    else
      Warn();
    break;

  case 5:
    if (msg_type_ == UBX_RXM_SFRBX)
      qzss_num_sfrbx_++;
    else
      Warn();
    break;

  case 6:
    if (msg_type_ == UBX_RXM_SFRBX)
      glonass_num_sfrbx_++;
    else
      Warn();
    break;

  default:
    Warn();
    break;
  }
}

void GalileoParser::GnssCount(SignalInformation &payload) {
  switch (payload.gnssId) {

  case 0:
    if (msg_type_ == UBX_NAV_SIG)
      gps_num_navsig_++;
    else
      Warn();
    break;

  case 1:
    if (msg_type_ == UBX_NAV_SIG)
      sbas_num_navsig_++;
    else
      Warn();
    break;

  case 2:
    if (msg_type_ == UBX_NAV_SIG)
      galileo_num_navsig_++;
    else
      Warn();
    break;

  case 3:
    if (msg_type_ == UBX_NAV_SIG)
      beidou_num_navsig_++;
    else
      Warn();
    break;

  case 5:
    if (msg_type_ == UBX_NAV_SIG)
      qzss_num_navsig_++;
    else
      Warn();
    break;

  case 6:
    if (msg_type_ == UBX_NAV_SIG)
      glonass_num_navsig_++;
    else
      Warn();
    break;

  default:
    Warn();
    break;
  }
}

unsigned int GalileoParser::GetBits(unsigned int x, int n) {
  unsigned int res =  (x >> pos_) & ~(~0 << n);
  pos_ += n;
  return res;
}

void GalileoParser::Log() const {

  std::cout << "UBX-RXM-SFRBX: " << rxm_sfrbx_counter << std::endl;
  std::cout << "\nGalileo: " << galileo_num_sfrbx_
            << "\nGPS: " << gps_num_sfrbx_
            << "\nGLONASS: " << glonass_num_sfrbx_
            << "\nBeidou: " << beidou_num_sfrbx_
            << "\nQZSS: " << qzss_num_sfrbx_ << "\nSBAS: " << sbas_num_sfrbx_
            << std::endl;

  std::cout << "\nUBX-NAV-SIG: " << nav_sig_counter << std::endl;
  std::cout << "\nGalileo: " << galileo_num_navsig_
            << "\nGPS: " << gps_num_navsig_
            << "\nGLONASS: " << glonass_num_navsig_
            << "\nBeidou: " << beidou_num_navsig_
            << "\nQZSS: " << qzss_num_navsig_ << "\nSBAS: " << sbas_num_navsig_
            << std::endl;

  std::cout << "\nCounter: " << counter << std::endl;
  std::cout << "True: " << true_counter << std::endl;
  std::cout << "False: " << false_counter << std::endl;
}

void GalileoParser::Warn() const { std::cout << "WARNING!!!" << std::endl; }