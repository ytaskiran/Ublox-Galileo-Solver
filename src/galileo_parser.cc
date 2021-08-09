#include "galileo_parser.h"

GalileoParser::GalileoParser(const std::string& path) : file_(path) {}

void GalileoParser::Read() {
  raw_data_.open(file_, std::ios::binary);

  if (!raw_data_.is_open()) {
    std::cout << "File cannot be read" << std::endl;
  }

  while (!raw_data_.eof()) {
    raw_data_.read(reinterpret_cast<char*>(&byte_), sizeof(byte_));

    CheckSyncHeaders(byte_);

    if (sync_lock_1_ && sync_lock_2_) {
      ParseInitialData(raw_data_, msg_type);
      ParsePayloadData(raw_data_);

      sync_lock_1_ = false;
      sync_lock_2_ = false;
    }
  }
  Log();
  raw_data_.close();
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

void GalileoParser::ParseInitialData(std::ifstream& raw_data_, MessageType& msg_type) {
  raw_data_.read(reinterpret_cast<char*>(&msg_data),
                 sizeof(msg_data));

  if (msg_data.message_class == 0x02 && msg_data.message_id == 0x13){
    msg_type = UBX_RXM_SFRBX;
    rxm_sfrbx_counter++;
  }
  else if (msg_data.message_class == 0x01 && msg_data.message_id == 0x43){
    msg_type = UBX_NAV_SIG;
    nav_sig_counter++;
  } 
  else msg_type = NOT_DEFINED;
}

void GalileoParser::ParsePayloadData(std::ifstream& raw_data_) {
  if (msg_type == UBX_RXM_SFRBX){
    raw_data_.read(reinterpret_cast<char*>(&payload_sfrbx_head), sizeof(payload_sfrbx_head));
    GnssCount(payload_sfrbx_head);
  }
  
  else if (msg_type == UBX_NAV_SIG){
    raw_data_.read(reinterpret_cast<char*>(&payload_navsig_head), sizeof(payload_navsig_head));
    
    for (int i=0; i<payload_navsig_head.numSigs; i++){      
      raw_data_.read(reinterpret_cast<char*>(&payload_navsig), sizeof(payload_navsig));
      GnssCount(payload_navsig);
    }
  }
}

void GalileoParser::GnssCount(NavigationDataHead& payload){
  switch (payload.gnssId){

    case 0:
      if (msg_type == UBX_RXM_SFRBX) gps_num_sfrbx_++;
      else Warn();
      break;

    case 1:
      if (msg_type == UBX_RXM_SFRBX) sbas_num_sfrbx_++;
      else Warn();
      break;

    case 2:
      if (msg_type == UBX_RXM_SFRBX) galileo_num_sfrbx_++;
      else Warn();
      break;

    case 3:
      if (msg_type == UBX_RXM_SFRBX) beidou_num_sfrbx_++;
      else Warn();
      break;

    case 5:
      if (msg_type == UBX_RXM_SFRBX) qzss_num_sfrbx_++;
      else Warn();
      break;

    case 6:
      if (msg_type == UBX_RXM_SFRBX) glonass_num_sfrbx_++;
      else Warn();
      break;

    default:
      std::cout << "WARNING!!!" << std::endl;
      break;
  } 
}


void GalileoParser::GnssCount(SignalInformation& payload){
  switch (payload.gnssId){

    case 0:
      if (msg_type == UBX_NAV_SIG) gps_num_navsig_++;
      else Warn();
      break;

    case 1:
      if (msg_type == UBX_NAV_SIG) sbas_num_navsig_++;
      else Warn();
      break;

    case 2:
      if (msg_type == UBX_NAV_SIG) galileo_num_navsig_++;
      else Warn();
      break;

    case 3:
      if (msg_type == UBX_NAV_SIG) beidou_num_navsig_++;
      else Warn();
      break;

    case 5:
      if (msg_type == UBX_NAV_SIG) qzss_num_navsig_++;
      else Warn();
      break;

    case 6:
      if (msg_type == UBX_NAV_SIG) glonass_num_navsig_++;
      else Warn();
      break;

    default:
      Warn();
      break;
  }
}


void GalileoParser::Log(){

  std::cout << "UBX-RXM-SFRBX: " << rxm_sfrbx_counter << std::endl;
  std::cout << "\nGalileo: " << galileo_num_sfrbx_ 
            << "\nGPS: " << gps_num_sfrbx_
            << "\nGLONASS: " << glonass_num_sfrbx_
            << "\nBeidou: " << beidou_num_sfrbx_
            << "\nQZSS: " << qzss_num_sfrbx_
            << "\nSBAS: " << sbas_num_sfrbx_ 
            << std::endl;  

  std::cout << "\nUBX-NAV-SIG: " << nav_sig_counter << std::endl;
  std::cout << "\nGalileo: " << galileo_num_navsig_ 
            << "\nGPS: " << gps_num_navsig_
            << "\nGLONASS: " << glonass_num_navsig_
            << "\nBeidou: " << beidou_num_navsig_
            << "\nQZSS: " << qzss_num_navsig_
            << "\nSBAS: " << sbas_num_navsig_ 
            << std::endl;  
}

void GalileoParser::Warn() {
  std::cout << "WARNING!!!" << std::endl;
}