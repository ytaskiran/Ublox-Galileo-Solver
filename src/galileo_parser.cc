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

    unsigned int dword = GetDataWord<unsigned int>();

    payload_data_word_head.even_odd = GetBits(dword, 1);
    payload_data_word_head.page_type = GetBits(dword, 1);
    payload_data_word_head.word_type = GetBits(dword, 6);

    if (payload_data_word_head.page_type == 1) // Skip alert pages
      return false;

    counter++;
    even_ = payload_data_word_head.even_odd;

    if (!DetermineWordType(payload_data_word_head))
      return false;

    if (!ParseDataWord(raw_data_, dword))
      return false;

    true_counter++;

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
    word_type_ = FEC2;
    return true;

  case 18:
    word_type_ = FEC2;
    return true;

  case 19:
    word_type_ = FEC2;
    return true;

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

bool GalileoParser::ParseDataWord(std::ifstream &raw_data_,
                                  unsigned int dword_1) {
  if (word_type_ == EPHEMERIS_1) {

    word_type_1.issue_of_data = GetBits(dword_1, 10);
    word_type_1.reference_time = GetBits(dword_1, 14);

    unsigned int dword_2 = GetDataWord<unsigned int>();
    word_type_1.mean_anomaly = GetBits(dword_2, 32);

    unsigned int dword_3 = GetDataWord<unsigned int>();
    word_type_1.eccentricity = GetBits(dword_3, 32);

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();
    
    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    if (word_util.tail != 0) {
      false_counter++;
      return false;
    }

    if (even_ == 0) {
      if (word_util.even_odd != 1){
        false_counter++;
        return false;
      }
    }
    if (even_ == 1) {
      if (word_util.even_odd != 0){
        false_counter++;
        return false;
      }
    }

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    word_type_1.root_semi_major_axis = GetBits(dword_data, 32);
    word_type_1.reserved = GetBits(dword_data, 2); 

    return true;
  }

  else if (word_type_ == EPHEMERIS_2) {

    word_type_2.issue_of_data = GetBits(dword_1, 10);
    signed longitude_1 = GetBits(dword_1, 14);

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed longitude_2 = GetBits(dword_2, 18);
    signed inclination_angle_1 = GetBits(dword_2, 14);
    
    word_type_2.longitude = ConcatenateBits(longitude_1, longitude_2, 14, 18);

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed inclination_angle_2 = GetBits(dword_3, 18);
    signed perigee_1 = GetBits(dword_3, 14);

    word_type_2.inclination_angle = ConcatenateBits(inclination_angle_1,
                                                    inclination_angle_2,
                                                    14,
                                                    18);

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();
    
    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);


    if (word_util.tail != 0) {
      false_counter++;
      return false;
    }

    if (even_ == 0) {
      if (word_util.even_odd != 1){
        false_counter++;
        return false;
      }
    }
    if (even_ == 1) {
      if (word_util.even_odd != 0){
        false_counter++;
        return false;
      }
    }

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed perigee_2 = GetBits(dword_data, 18);
    word_type_2.ia_rate_of_change = GetBits(dword_data, 14);
    word_type_2.reserved = GetBits(dword_data, 2); 

    word_type_2.perigee = ConcatenateBits(perigee_1,
                                          perigee_2,
                                          14,
                                          18);

    return true;
  }


  else if (word_type_ == EPHEMERIS_3) {

    unsigned issue_of_data = GetBits(dword_1, 10);
    signed ra_rate_of_change_1 = GetBits(dword_1, 14);

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed ra_rate_of_change_2 = GetBits(dword_2, 10);
    signed mean_motion_difference = GetBits(dword_2, 16);
    signed C_uc_1 = GetBits(dword_2, 6);

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed C_uc_2 = GetBits(dword_3, 10);
    signed C_us = GetBits(dword_3, 16);
    signed C_rc_1 = GetBits(dword_3, 6);

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();
    
    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed C_rc_2 = GetBits(dword_data, 10);
    signed C_rs = GetBits(dword_data, 16);
    unsigned sisa = GetBits(dword_data, 8);

    word_type_3.issue_of_data = issue_of_data;
    word_type_3.ra_rate_of_change = ConcatenateBits(ra_rate_of_change_1,
                                                    ra_rate_of_change_2,
                                                    14,
                                                    10);
    word_type_3.mean_motion_difference = mean_motion_difference;
    word_type_3.C_uc = ConcatenateBits(C_uc_1, 
                                       C_uc_2,
                                       6,
                                       10);
    word_type_3.C_us = C_us;
    word_type_3.C_rc = ConcatenateBits(C_rc_1,
                                       C_rc_2,
                                       6,
                                       10);
    word_type_3.C_rs = C_rs;
    word_type_3.sisa = sisa;

    return true;
  }


  else if (word_type_ == EPHEMERIS_4__CLOCK_CORRECTION) {

    unsigned issue_of_data = GetBits(dword_1, 10);
    unsigned svid = GetBits(dword_1, 6);
    signed C_ic_1 = GetBits(dword_1, 8); // 8 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed C_ic_2 = GetBits(dword_2, 8);
    signed C_is = GetBits(dword_2, 16);
    unsigned reference_1 = GetBits(dword_2, 8); // 6 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    unsigned reference_2 = GetBits(dword_3, 6);
    signed clock_bias_corr_1 = GetBits(dword_3, 26); // 5 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();
    
    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed clock_bias_corr_2 = GetBits(dword_data, 5);
    signed clock_drift_corr = GetBits(dword_data, 21);
    signed clock_drift_rate_corr = GetBits(dword_data, 6);
    unsigned spare = GetBits(dword_data, 2);

    word_type_4.issue_of_data = issue_of_data;
    word_type_4.svid = svid;
    word_type_4.C_ic = ConcatenateBits(C_ic_1, C_ic_2, 8, 8);
    word_type_4.C_is = C_is;
    word_type_4.reference = ConcatenateBits(reference_1, reference_2, 8, 6);
    word_type_4.clock_bias_corr = ConcatenateBits(clock_bias_corr_1,
                                                  clock_bias_corr_2, 
                                                  26, 5);
    word_type_4.clock_drift_corr = clock_drift_corr;
    word_type_4.clock_drift_rate_corr = clock_drift_rate_corr;
    word_type_4.spare = spare;

    return true;
  }


  else if (word_type_ == IONOSPHERIC_CORRECTION__BGD__SIG_HEALTH__DVS__GST) {

    unsigned effionl_0 = GetBits(dword_1, 11);
    signed effionl_1 = GetBits(dword_1, 11);
    signed effionl_2_1 = GetBits(dword_1, 2); // 12 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed effionl_2_2 = GetBits(dword_2, 12);
    unsigned region1 = GetBits(dword_2, 1);
    unsigned region2 = GetBits(dword_2, 1);
    unsigned region3 = GetBits(dword_2, 1);
    unsigned region4 = GetBits(dword_2, 1);
    unsigned region5 = GetBits(dword_2, 1);
    signed bgd_1 = GetBits(dword_2, 10);
    signed bgd_2_1 = GetBits(dword_2, 5); // 5 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed bgd_2_2 = GetBits(dword_3, 5);
    unsigned sig_health_e5b = GetBits(dword_3, 2);
    unsigned sig_health_e1 = GetBits(dword_3, 2);
    unsigned data_validity_e5b = GetBits(dword_3, 1);
    unsigned data_validity_e1 = GetBits(dword_3, 1);
    unsigned week_num = GetBits(dword_3, 12);
    unsigned time_of_week_1 = GetBits(dword_3, 9); // 11 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    unsigned time_of_week_2 = GetBits(dword_data, 11);
    unsigned spare = GetBits(dword_data, 11);

    word_type_5.effionl_0 = effionl_0;
    word_type_5.effionl_1 = effionl_1;
    word_type_5.effionl_2 = ConcatenateBits(effionl_2_1, effionl_2_2, 2, 12);
    word_type_5.region1 = region1;
    word_type_5.region2 = region2;
    word_type_5.region3 = region3;
    word_type_5.region4 = region4;
    word_type_5.region5 = region5;
    word_type_5.bgd_1 = bgd_1;
    word_type_5.bgd_2 = ConcatenateBits(bgd_2_1, bgd_2_2, 5, 5);
    word_type_5.sig_health_e5b = sig_health_e5b;
    word_type_5.sig_health_e1 = sig_health_e1;
    word_type_5.data_validity_e5b = data_validity_e5b;
    word_type_5.data_validity_e1 = data_validity_e1;
    word_type_5.week_num = week_num;
    word_type_5.time_of_week = ConcatenateBits(time_of_week_1, time_of_week_2, 9, 11);
    word_type_5.spare = spare;

    return true;
  }


  else if (word_type_ == GST_UTC_CONVERSION) {
    
    signed A0_1 = GetBits(dword_1, 24); // 8 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed A0_2 = GetBits(dword_2, 8);
    signed A1 = GetBits(dword_2, 24);

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed ls_count_before = GetBits(dword_3, 8);
    unsigned utc_reference_tow = GetBits(dword_3, 8);
    unsigned utc_reference_week = GetBits(dword_3, 8);
    unsigned WN_lsf = GetBits(dword_3, 8);

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    unsigned day_num = GetBits(dword_data, 3);
    signed ls_count_after = GetBits(dword_data, 8);
    unsigned time_of_week = GetBits(dword_data, 20);
    unsigned spare = GetBits(dword_data, 3);


    word_type_6.A0 = ConcatenateBits(A0_1, A0_2, 24, 8);
    word_type_6.A1 = A1;
    word_type_6.ls_count_before = ls_count_before;
    word_type_6.utc_reference_tow = utc_reference_tow;
    word_type_6.utc_reference_week = utc_reference_week;
    word_type_6.WN_lsf = WN_lsf;
    word_type_6.day_num = day_num;
    word_type_6.ls_count_after = ls_count_after;
    word_type_6.time_of_week = time_of_week;
    word_type_6.spare = spare;

    return true;
  }


  else if (word_type_ == ALMANAC_1) {

    unsigned issue_of_data = GetBits(dword_1, 4);
    unsigned week_num = GetBits(dword_1, 2);
    unsigned ref_time = GetBits(dword_1, 10);
    unsigned svid_1 = GetBits(dword_1, 6);
    signed delta_root_a_1 = GetBits(dword_1, 2); // 11 left 

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed delta_root_a_2 = GetBits(dword_2, 11);
    unsigned eccentricity = GetBits(dword_2, 11);
    signed perigee_1 = GetBits(dword_2, 10); // 6 left 

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed perigee_2 = GetBits(dword_3, 6);
    signed diff_ia_na = GetBits(dword_3, 11);
    signed longitude_1 = GetBits(dword_3, 15); // 1 left 

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed longitude_2 = GetBits(dword_data, 1);
    signed roc_ra = GetBits(dword_data, 11);
    signed mean_anomaly = GetBits(dword_data, 16);
    unsigned reserved = GetBits(dword_data, 6);


    word_type_7.issue_of_data = issue_of_data;
    word_type_7.week_num = week_num;
    word_type_7.ref_time = ref_time;
    word_type_7.svid_1 = svid_1;
    word_type_7.delta_root_a = ConcatenateBits(delta_root_a_1, delta_root_a_2, 2, 11);
    word_type_7.eccentricity = eccentricity;
    word_type_7.perigee = ConcatenateBits(perigee_1, perigee_2, 10, 6);
    word_type_7.diff_ia_na = diff_ia_na;
    word_type_7.longitude = ConcatenateBits(longitude_1, longitude_2, 15, 1);
    word_type_7.roc_ra = roc_ra;
    word_type_7.mean_anomaly = mean_anomaly;
    word_type_7.reserved = reserved;

    return true;
  }


  else if (word_type_ == ALMANAC_2) {

    unsigned issue_of_data = GetBits(dword_1, 4);
    signed clock_corr_bias = GetBits(dword_1, 16);
    signed clock_corr_linear_1 = GetBits(dword_1, 4); // 9 left 

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed clock_corr_linear_2 = GetBits(dword_2, 9);
    unsigned sig_health_e5b = GetBits(dword_2, 2);
    unsigned sig_health_e1 = GetBits(dword_2, 2);
    unsigned svid_2 = GetBits(dword_2, 6);
    signed delta_root_a = GetBits(dword_2, 13);

    unsigned int dword_3 = GetDataWord<unsigned int>();
    unsigned eccentricity = GetBits(dword_3, 11);
    signed perigee = GetBits(dword_3, 16);
    signed diff_ia_na_1 = GetBits(dword_3, 5); // 6 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed diff_ia_na_2 = GetBits(dword_data, 6);
    signed longitude = GetBits(dword_data, 16);
    signed roc_ra = GetBits(dword_data, 11);
    unsigned spare = GetBits(dword_data, 1);


    word_type_8.issue_of_data = issue_of_data;
    word_type_8.clock_corr_bias = clock_corr_bias;
    word_type_8.clock_corr_linear = ConcatenateBits(clock_corr_linear_1, clock_corr_linear_2, 4, 9);
    word_type_8.sig_health_e5b = sig_health_e5b;
    word_type_8.sig_health_e1 = sig_health_e1;
    word_type_8.svid_2 = svid_2;
    word_type_8.delta_root_a = delta_root_a;
    word_type_8.eccentricity = eccentricity;
    word_type_8.perigee = perigee;
    word_type_8.diff_ia_na = ConcatenateBits(diff_ia_na_1, diff_ia_na_2, 5, 6);
    word_type_8.longitude = longitude;
    word_type_8.roc_ra = roc_ra;
    word_type_8.spare = spare;

    return true;
  }


  else if (word_type_ == ALMANAC_3) {

    unsigned issue_of_data = GetBits(dword_1, 4);
    unsigned week_num = GetBits(dword_1, 2);
    unsigned ref_time = GetBits(dword_1, 10);
    signed mean_anomaly_1 = GetBits(dword_1, 8); // 8 left 

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed mean_anomaly_2 = GetBits(dword_2, 8);
    signed clock_corr_bias = GetBits(dword_2, 16);
    signed clock_corr_linear_1 = GetBits(dword_2, 8); // 5 left 

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed clock_corr_linear_2 = GetBits(dword_3, 5);
    unsigned sig_health_e5b = GetBits(dword_3, 2);
    unsigned sig_health_e1 = GetBits(dword_3, 2);
    unsigned svid_3 = GetBits(dword_3, 6);
    signed delta_root_a = GetBits(dword_3, 13);
    unsigned eccentricity_1 = GetBits(dword_3, 4); // 7 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    unsigned eccentricity_2 = GetBits(dword_data, 7);
    signed perigee = GetBits(dword_data, 16);
    signed diff_ia_na = GetBits(dword_data, 11);


    word_type_9.issue_of_data = issue_of_data;
    word_type_9.week_num = week_num;
    word_type_9.ref_time = ref_time;
    word_type_9.mean_anomaly = ConcatenateBits(mean_anomaly_1, mean_anomaly_2, 8, 8);
    word_type_9.clock_corr_bias = clock_corr_bias;
    word_type_9.clock_corr_linear = ConcatenateBits(clock_corr_linear_1, clock_corr_linear_2, 8, 5);
    word_type_9.sig_health_e5b = sig_health_e5b;
    word_type_9.sig_health_e1 = sig_health_e1;
    word_type_9.svid_3 = svid_3;
    word_type_9.delta_root_a = delta_root_a;
    word_type_9.eccentricity = ConcatenateBits(eccentricity_1, eccentricity_2, 4, 7);
    word_type_9.perigee = perigee;
    word_type_9.diff_ia_na = diff_ia_na;

    return true;
  }

  else if (word_type_ == ALMANAC_4) {

    unsigned issue_of_data = GetBits(dword_1, 4);
    signed longitude = GetBits(dword_1, 16);
    signed roc_ra_1 = GetBits(dword_1, 4); // 7 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed roc_ra_2 = GetBits(dword_2, 7);
    signed mean_anomaly = GetBits(dword_2, 16);
    signed clock_corr_bias_1 = GetBits(dword_2, 9); // 7 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed clock_corr_bias_2 = GetBits(dword_3, 7);
    signed clock_corr_linear = GetBits(dword_3, 13);
    unsigned sig_health_e5b = GetBits(dword_3, 2);
    unsigned sig_health_e1 = GetBits(dword_3, 2);
    signed const_term_offset_1 = GetBits(dword_3, 8); // 8 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed const_term_offset_2 = GetBits(dword_data, 8);
    signed roc_offset = GetBits(dword_data, 12);
    unsigned ref_time = GetBits(dword_data, 8);
    unsigned week_num = GetBits(dword_data, 6);


    word_type_10.issue_of_data = issue_of_data;
    word_type_10.longitude = longitude;
    word_type_10.roc_ra = ConcatenateBits(roc_ra_1, roc_ra_2, 4, 7);
    word_type_10.mean_anomaly = mean_anomaly;
    word_type_10.clock_corr_bias = ConcatenateBits(clock_corr_bias_1, clock_corr_bias_2, 9, 7);
    word_type_10.clock_corr_linear = clock_corr_linear;
    word_type_10.sig_health_e5b = sig_health_e5b;
    word_type_10.sig_health_e1 = sig_health_e1;
    word_type_10.const_term_offset = ConcatenateBits(const_term_offset_1, const_term_offset_2, 8, 8);
    word_type_10.roc_offset = roc_offset;
    word_type_10.ref_time = ref_time;
    word_type_10.week_num = week_num;

    return true;
  }


  else if (word_type_ == REDUCED_CED) {

    signed delta_rced_smajor = GetBits(dword_1, 5);
    signed eccentricity_rced_x = GetBits(dword_1, 13);
    signed eccentricity_rced_y_1 = GetBits(dword_1, 6); // 7 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    signed eccentricity_rced_y_2 = GetBits(dword_2, 7);
    signed delta_rced_inclination = GetBits(dword_2, 17);
    signed rced_longitude_1 = GetBits(dword_2, 8); // 15 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    signed rced_longitude_2 = GetBits(dword_3, 15);
    signed lambda_rced_1 = GetBits(dword_3, 17); // 6 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    signed lambda_rced_2 = GetBits(dword_data, 6);
    signed rced_clock_corr_bias = GetBits(dword_data, 22);
    signed rced_clock_corr_drift = GetBits(dword_data, 6);


    word_type_16.delta_rced_smajor = delta_rced_smajor;
    word_type_16.eccentricity_rced_x = eccentricity_rced_x;
    word_type_16.eccentricity_rced_y = ConcatenateBits(eccentricity_rced_y_1, eccentricity_rced_y_2, 6, 7);
    word_type_16.delta_rced_inclination = delta_rced_inclination;
    word_type_16.rced_longitude = ConcatenateBits(rced_longitude_1, rced_longitude_2, 8, 15);
    word_type_16.lambda_rced = ConcatenateBits(lambda_rced_1, lambda_rced_2, 17, 6);
    word_type_16.rced_clock_corr_bias = rced_clock_corr_bias;
    word_type_16.rced_clock_corr_drift = rced_clock_corr_drift;

    return true;

  }

  else if (word_type_ == FEC2) {

    unsigned fec2_1 = GetBits(dword_1, 8);
    unsigned lsb = GetBits(dword_1, 2);
    unsigned long long fec2_2_1 = GetBits(dword_1, 14); // 50 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    unsigned long long fec2_2_2 = GetBits(dword_2, 32); // 18 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    unsigned long long fec2_2_3 = GetBits(dword_3, 18);
    unsigned long long fec2_3_1 = GetBits(dword_3, 14); // 34 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    unsigned long long fec2_3_2 = GetBits(dword_data, 34);

    fec2_2_2 = ConcatenateBits(fec2_2_2, fec2_2_3, 32, 18);

    word_type_17.fec2_1 = fec2_1;
    word_type_17.lsb = lsb;
    word_type_17.fec2_2 = ConcatenateBits(fec2_2_1, fec2_2_2, 14, 50);
    word_type_17.fec2_3 = ConcatenateBits(fec2_3_1, fec2_3_2, 14, 34);

    return true;
  }


  else if (word_type_ == SPARE){

    unsigned time = GetBits(dword_1, 2);
    unsigned long long spare_1 = GetBits(dword_1, 22); // 42 left

    unsigned int dword_2 = GetDataWord<unsigned int>();
    unsigned long long spare_2 = GetBits(dword_2, 32); // 10 left

    unsigned int dword_3 = GetDataWord<unsigned int>();
    unsigned long long spare_3 = GetBits(dword_3, 10);
    unsigned spare2_1 = GetBits(dword_3, 22); // 2 left

    unsigned long long dword_middle = GetWordMiddle<unsigned long long>();

    unsigned long long dword_util = dword_middle;
    MaskWordUtilMiddle(dword_util);
    word_util.tail = GetBits(dword_util, 6);
    word_util.even_odd = GetBits(dword_util, 1);
    word_util.page_type = GetBits(dword_util, 1);

    unsigned long long dword_data = dword_middle;
    MaskWordDataMiddle(dword_data);
    unsigned spare2_2 = GetBits(dword_data, 2);
    unsigned week_num = GetBits(dword_data, 12);
    unsigned time_of_week = GetBits(dword_data, 20);

    spare_2 = ConcatenateBits(spare_2, spare_3, 32, 10);

    word_type_0.time = time;
    word_type_0.spare = ConcatenateBits(spare_1, spare_2, 22, 42);
    word_type_0.spare2 = ConcatenateBits(spare2_1, spare2_2, 22, 2);
    word_type_0.week_num = week_num;
    word_type_0.time_of_week = time_of_week;

    return true;
  }

  else if (word_type_ == DUMMY){
    return true;
  }

  else {
    return false;
  }
}

template <typename T> T GalileoParser::GetDataWord() {
  pos_ = 0;
  char * buffer = new char [4];
  raw_data_.read(buffer, 4);

  T *dword_ref = reinterpret_cast<T *>(buffer);
  T dword = *dword_ref;

  ConvertBits<T>(dword);

  delete[] buffer;

  return dword;
}

template <typename T> T GalileoParser::GetWordMiddle() {
  char * buffer = new char [8];
  raw_data_.read(buffer, 8);

  T *dword_middle_ref = reinterpret_cast<T *>(buffer);
  T dword_middle = *dword_middle_ref;

  ConvertBits<T>(dword_middle);

  delete[] buffer;

  return dword_middle;
}

template <typename T> void GalileoParser::MaskWordUtilMiddle(T& dword_util) {
  pos_ = 0;
  dword_util = dword_util & mask1_;
  dword_util = (dword_util << 18) | (dword_util << 26);
}

template <typename T> void GalileoParser::MaskWordDataMiddle(T& dword_data) {
  pos_ = 0;
  dword_data = dword_data & mask2_;
  dword_data = (dword_data) | (dword_data << 16);
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

template <typename T> void GalileoParser::ConvertBits(T &x) {
  if (sizeof(x) == 4)
    x =
        (x << 24) | ((x << 8) & 0xFF0000) | ((x >> 8) & 0xFF00) | (x >> 24);

  else if (sizeof(x) == 8)
    x = (x << 56) | ((x << 40) & 0xFF000000000000) |
         ((x << 24) & 0xFF0000000000) | ((x << 8) & 0xFF00000000) |
         ((x >> 8) & 0xFF000000) | ((x >> 24) & 0xFF0000) |
         ((x >> 40) & 0xFF00) | (x >> 56);
}

template <typename T> T GalileoParser::GetBits(T x, int n) {
  T res = (x << pos_) & (~0 << ((sizeof(x) * 8) - n));
  res = (res >> ((sizeof(x) * 8) - n));
  pos_ += n;
  return res;
}

template <typename T>
T GalileoParser::ConcatenateBits(T data1, T data2, int size1, int size2) {
  T data = (data1 << size2) | (data2);
  return data;
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