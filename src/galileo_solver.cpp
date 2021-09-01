#include "galileo_solver.h"


GalileoSolver::GalileoSolver(const std::string &path) : file_(path) {}


void GalileoSolver::read() 
{
  raw_data_.open(file_, std::ios::binary);

  if (!raw_data_.is_open()) 
    std::cout << "File cannot be read" << std::endl;

  while (!raw_data_.eof()) 
  {
    raw_data_.read(reinterpret_cast<char *>(&byte_), sizeof(byte_));

    checkSyncHeaders(byte_);

    if (sync_lock_1_ && sync_lock_2_) 
    {
      parseInitialData(raw_data_);
      parsePayloadData(raw_data_);
      pos_ = 0;
      bitsize_ = 32;

      sync_lock_1_ = false;
      sync_lock_2_ = false;
    }
  }
  log();
  raw_data_.close();
}


void GalileoSolver::checkSyncHeaders(uint8_t &byte_) 
{
  if (!sync_lock_1_) 
  {
    if (byte_ == SYNC_HEADER_1_) 
      sync_lock_1_ = true;
  } 
  
  else 
  {
    if (byte_ == SYNC_HEADER_2_)
      sync_lock_2_ = true;

    else
      sync_lock_1_ = false;
  }
}


void GalileoSolver::parseInitialData(std::ifstream &raw_data_) 
{ 
  raw_data_.read(reinterpret_cast<char *>(&msg_head), sizeof(msg_head));

  if (msg_head.message_class == 0x02 && msg_head.message_id == 0x13) 
  {
    msg_type_ = UBX_RXM_SFRBX;
    rxm_sfrbx_counter++;
  } 

  else if (msg_head.message_class == 0x01 && msg_head.message_id == 0x43) 
  {
    msg_type_ = UBX_NAV_SIG;
    nav_sig_counter++;
  } 

  else
    msg_type_ = NOT_DEFINED;
}


bool GalileoSolver::checkSum(std::ifstream &raw_data_)
{
  raw_data_.seekg(-4, std::ios::cur);

  char *buffer = new char[msg_head.length + 4];
  raw_data_.read(buffer, (msg_head.length + 4));

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (int i=0; i<(msg_head.length+4); i++) 
  {
    ck_a = ck_a + buffer[i];
    ck_b = ck_b + ck_a;
  }

  raw_data_.read(reinterpret_cast<char *>(&checksum), sizeof(checksum));
  raw_data_.seekg(-(msg_head.length + 2), std::ios::cur);

  delete[] buffer;

  if (ck_a == checksum.ck_a && ck_b == checksum.ck_b) 
    return true;

  else 
  {
    std::cout << "checksum false" << std::endl; 
    return false;
  }
}

bool GalileoSolver::parsePayloadData(std::ifstream &raw_data_) 
{
  if (msg_type_ == UBX_RXM_SFRBX) 
  {
    
    if (!checkSum(raw_data_)) {false_counter++; return false;}

    raw_data_.read(reinterpret_cast<char *>(&payload_sfrbx_head),
                   sizeof(payload_sfrbx_head));

    gnssCount(payload_sfrbx_head);

    if (payload_sfrbx_head.gnssId != 2)
      return false;


    uint32_t dword = getDataWord();

    payload_data_word_head.even_odd = getBits(dword, 1);
    payload_data_word_head.page_type = getBits(dword, 1);
    payload_data_word_head.word_type = getBits(dword, 6);

    if (payload_data_word_head.page_type == 1) // Skip alert pages
      return false;

    svId_ = payload_sfrbx_head.svId;
    sigId_ = payload_sfrbx_head.reserved0;

    counter++;
    even_ = payload_data_word_head.even_odd;

    classifySvid();

    if (!determineWordType(payload_data_word_head))
      return false;

    if (!parseDataWord(raw_data_, dword))
      return false;

    true_counter++;

    return true;
  }

  else if (msg_type_ == UBX_NAV_SIG) 
  {
    raw_data_.read(reinterpret_cast<char *>(&payload_navsig_head),
                   sizeof(payload_navsig_head));

    for (int i = 0; i < payload_navsig_head.numSigs; i++) 
    {
      raw_data_.read(reinterpret_cast<char *>(&payload_navsig),
                     sizeof(payload_navsig));
      gnssCount(payload_navsig);
    }

    return true;

  }

  return false;
}


bool GalileoSolver::determineWordType(MessageDataWordHead &payload_data_word_head)
{
  switch (payload_data_word_head.word_type) 
  {
  case 0:
    word_type_ = SPARE;
    ++wordtype0_counter;
    return true;

  case 1:
    word_type_ = EPHEMERIS_1;
    ++wordtype1_counter;
    return true;

  case 2:
    word_type_ = EPHEMERIS_2;
    ++wordtype2_counter;
    return true;

  case 3:
    word_type_ = EPHEMERIS_3;
    ++wordtype3_counter;
    return true;

  case 4:
    word_type_ = EPHEMERIS_4__CLOCK_CORRECTION;
    ++wordtype4_counter;
    return true;

  case 5:
    word_type_ = IONOSPHERIC_CORRECTION__BGD__SIG_HEALTH__DVS__GST;
    ++wordtype5_counter;
    return true;

  case 6:
    word_type_ = GST_UTC_CONVERSION;
    ++wordtype6_counter;
    return true;

  case 7:
    word_type_ = ALMANAC_1;
    ++wordtype7_counter;
    return true;

  case 8:
    word_type_ = ALMANAC_2;
    ++wordtype8_counter;
    return true;

  case 9:
    word_type_ = ALMANAC_3;
    ++wordtype9_counter;
    return true;

  case 10:
    word_type_ = ALMANAC_4;
    ++wordtype10_counter;
    return true;

  case 16:
    word_type_ = REDUCED_CED;
    ++wordtype16_counter;
    return true;

  case 17:
    word_type_ = FEC2;
    ++wordtype17_counter;
    return true;

  case 18:
    word_type_ = FEC2;
    ++wordtype17_counter;
    return true;

  case 19:
    word_type_ = FEC2;
    ++wordtype17_counter;
    return true;

  case 20:
    word_type_ = FEC2;
    ++wordtype17_counter;
    return true;

  case 63:
    word_type_ = DUMMY;
    ++wordtype63_counter;
    return true;

  default:
    warn();
    false_counter++;
    return false;
  }
}


bool GalileoSolver::parseDataWord(std::ifstream &raw_data_, uint32_t dword_1)                              
{

  if (word_type_ == EPHEMERIS_1) // Word Type 1
  { 
    unsigned issue_of_data = getBits(dword_1, 10);
    unsigned reference_time = getBits(dword_1, 14);


    uint32_t dword_2 = getDataWord();
    signed mean_anomaly = getBits(dword_2, 32);


    uint32_t dword_3 = getDataWord();
    unsigned eccentricity = getBits(dword_3, 32);


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned root_semi_major_axis = getBits(dword_data, 32);
    unsigned reserved = getBits(dword_data, 2);


    word_type_1.issue_of_data = issue_of_data;
    word_type_1.reference_time = reference_time;
    word_type_1.mean_anomaly = mean_anomaly;
    word_type_1.eccentricity = eccentricity;
    word_type_1.root_semi_major_axis = root_semi_major_axis;
    word_type_1.reserved = reserved;


    nav_data[svId_-1].add(word_type_1, svId_, sigId_);

    
    return true;
  }


  else if (word_type_ == EPHEMERIS_2) // Word Type 2
  {
    unsigned issue_of_data = getBits(dword_1, 10);
    signed longitude_1 = getBits(dword_1, 14); // 18 left


    uint32_t dword_2 = getDataWord();
    signed longitude_2 = getBits(dword_2, 18);
    signed inclination_angle_1 = getBits(dword_2, 14); // 18 left


    uint32_t dword_3 = getDataWord();
    signed inclination_angle_2 = getBits(dword_3, 18);
    signed perigee_1 = getBits(dword_3, 14); // 18 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }
      

    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed perigee_2 = getBits(dword_data, 18);
    signed ia_rate_of_change = getBits(dword_data, 14);
    unsigned reserved = getBits(dword_data, 2);

    
    word_type_2.issue_of_data = issue_of_data;
    word_type_2.longitude = concatenateBits(longitude_1, longitude_2, 14, 18);
    word_type_2.inclination_angle = concatenateBits(inclination_angle_1, inclination_angle_2, 14, 18);
    word_type_2.perigee = concatenateBits(perigee_1, perigee_2, 14, 18);
    word_type_2.ia_rate_of_change = ia_rate_of_change;
    word_type_2.reserved = reserved;


    nav_data[svId_-1].add(word_type_2, svId_, sigId_);


    return true;
  }


  else if (word_type_ == EPHEMERIS_3) // Word Type 3
  { 
    unsigned issue_of_data = getBits(dword_1, 10);
    signed ra_rate_of_change_1 = getBits(dword_1, 14); // 10 left


    uint32_t dword_2 = getDataWord();
    signed ra_rate_of_change_2 = getBits(dword_2, 10);
    signed mean_motion_difference = getBits(dword_2, 16);
    signed C_uc_1 = getBits(dword_2, 6); // 10 left


    uint32_t dword_3 = getDataWord();
    signed C_uc_2 = getBits(dword_3, 10);
    signed C_us = getBits(dword_3, 16);
    signed C_rc_1 = getBits(dword_3, 6); // 10 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed C_rc_2 = getBits(dword_data, 10);
    signed C_rs = getBits(dword_data, 16);
    unsigned sisa = getBits(dword_data, 8);


    word_type_3.issue_of_data = issue_of_data;
    word_type_3.ra_rate_of_change = concatenateBits(ra_rate_of_change_1, ra_rate_of_change_2, 14, 10);
    word_type_3.mean_motion_difference = mean_motion_difference;
    word_type_3.C_uc = concatenateBits(C_uc_1, C_uc_2, 6, 10);
    word_type_3.C_us = C_us;
    word_type_3.C_rc = concatenateBits(C_rc_1, C_rc_2, 6, 10);
    word_type_3.C_rs = C_rs;
    word_type_3.sisa = sisa;


    nav_data[svId_-1].add(word_type_3, svId_, sigId_);


    return true;
  }


  else if (word_type_ == EPHEMERIS_4__CLOCK_CORRECTION) // Word Type 4
  { 
    unsigned issue_of_data = getBits(dword_1, 10);
    unsigned svid = getBits(dword_1, 6);
    signed C_ic_1 = getBits(dword_1, 8); // 8 left


    uint32_t dword_2 = getDataWord();
    signed C_ic_2 = getBits(dword_2, 8);
    signed C_is = getBits(dword_2, 16);
    unsigned reference_1 = getBits(dword_2, 8); // 6 left


    uint32_t dword_3 = getDataWord();
    unsigned reference_2 = getBits(dword_3, 6);
    signed clock_bias_corr_1 = getBits(dword_3, 26); // 5 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);
    

    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed clock_bias_corr_2 = getBits(dword_data, 5);
    signed clock_drift_corr = getBits(dword_data, 21);
    signed clock_drift_rate_corr = getBits(dword_data, 6);
    unsigned spare = getBits(dword_data, 2);


    word_type_4.issue_of_data = issue_of_data;
    word_type_4.svid = svid;
    word_type_4.C_ic = concatenateBits(C_ic_1, C_ic_2, 8, 8);
    word_type_4.C_is = C_is;
    word_type_4.reference = concatenateBits(reference_1, reference_2, 8, 6);
    word_type_4.clock_bias_corr = concatenateBits(clock_bias_corr_1, clock_bias_corr_2, 26, 5);
    word_type_4.clock_drift_corr = clock_drift_corr;
    word_type_4.clock_drift_rate_corr = clock_drift_rate_corr;
    word_type_4.spare = spare;


    nav_data[svId_-1].add(word_type_4, svId_, sigId_);


    return true;
  }


  else if (word_type_ == IONOSPHERIC_CORRECTION__BGD__SIG_HEALTH__DVS__GST) // Word Type 5
  {
    unsigned effionl_0 = getBits(dword_1, 11);
    signed effionl_1 = getBits(dword_1, 11);
    signed effionl_2_1 = getBits(dword_1, 2); // 12 left


    uint32_t dword_2 = getDataWord();
    signed effionl_2_2 = getBits(dword_2, 12);
    unsigned region1 = getBits(dword_2, 1);
    unsigned region2 = getBits(dword_2, 1);
    unsigned region3 = getBits(dword_2, 1);
    unsigned region4 = getBits(dword_2, 1);
    unsigned region5 = getBits(dword_2, 1);
    signed bgd_1 = getBits(dword_2, 10);
    signed bgd_2_1 = getBits(dword_2, 5); // 5 left


    uint32_t dword_3 = getDataWord();
    signed bgd_2_2 = getBits(dword_3, 5);
    unsigned sig_health_e5b = getBits(dword_3, 2);
    unsigned sig_health_e1 = getBits(dword_3, 2);
    unsigned data_validity_e5b = getBits(dword_3, 1);
    unsigned data_validity_e1 = getBits(dword_3, 1);
    unsigned week_num = getBits(dword_3, 12);
    unsigned time_of_week_1 = getBits(dword_3, 9); // 11 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned time_of_week_2 = getBits(dword_data, 11);
    unsigned spare = getBits(dword_data, 11);


    unsigned sig_hs_dvs_1 = concatenateBits(sig_health_e5b, data_validity_e5b, 2, 1);
    unsigned sig_hs_dvs_2 = concatenateBits(sig_hs_dvs_1, (unsigned int)0, 3, 3);
    unsigned sig_hs_dvs_3 = concatenateBits(sig_hs_dvs_2, sig_health_e1, 6, 2);
    unsigned sig_hs_dvs = concatenateBits(sig_hs_dvs_3, data_validity_e1, 8, 1);


    word_type_5.effionl_0 = effionl_0;
    word_type_5.effionl_1 = effionl_1;
    word_type_5.effionl_2 = concatenateBits(effionl_2_1, effionl_2_2, 2, 12);
    word_type_5.region1 = region1;
    word_type_5.region2 = region2;
    word_type_5.region3 = region3;
    word_type_5.region4 = region4;
    word_type_5.region5 = region5;
    word_type_5.bgd_1 = bgd_1;
    word_type_5.bgd_2 = concatenateBits(bgd_2_1, bgd_2_2, 5, 5);
    word_type_5.sig_health_e5b = sig_health_e5b;
    word_type_5.sig_health_e1 = sig_health_e1;
    word_type_5.data_validity_e5b = data_validity_e5b;
    word_type_5.data_validity_e1 = data_validity_e1;
    word_type_5.sig_health_validity = sig_hs_dvs;
    word_type_5.week_num = week_num;
    word_type_5.time_of_week = concatenateBits(time_of_week_1, time_of_week_2, 9, 11);
    word_type_5.spare = spare;


    nav_data[svId_-1].add(word_type_5, svId_, sigId_);


    return true;
  }


  else if (word_type_ == GST_UTC_CONVERSION) // Word Type 6
  {
    signed A0_1 = getBits(dword_1, 24); // 8 left


    uint32_t dword_2 = getDataWord();
    signed A0_2 = getBits(dword_2, 8);
    signed A1 = getBits(dword_2, 24);


    uint32_t dword_3 = getDataWord();
    signed ls_count_before = getBits(dword_3, 8);
    unsigned utc_reference_tow = getBits(dword_3, 8);
    unsigned utc_reference_week = getBits(dword_3, 8);
    unsigned WN_lsf = getBits(dword_3, 8);


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned day_num = getBits(dword_data, 3);
    signed ls_count_after = getBits(dword_data, 8);
    unsigned time_of_week = getBits(dword_data, 20);
    unsigned spare = getBits(dword_data, 3);


    word_type_6.A0 = concatenateBits(A0_1, A0_2, 24, 8);
    word_type_6.A1 = A1;
    word_type_6.ls_count_before = ls_count_before;
    word_type_6.utc_reference_tow = utc_reference_tow;
    word_type_6.utc_reference_week = utc_reference_week;
    word_type_6.WN_lsf = WN_lsf;
    word_type_6.day_num = day_num;
    word_type_6.ls_count_after = ls_count_after;
    word_type_6.time_of_week = time_of_week;
    word_type_6.spare = spare;


    nav_data[svId_-1].add(word_type_6, svId_, sigId_);


    return true;
  }


  else if (word_type_ == ALMANAC_1) // Word Type 7
  {
    unsigned issue_of_data = getBits(dword_1, 4);
    unsigned week_num = getBits(dword_1, 2);
    unsigned ref_time = getBits(dword_1, 10);
    unsigned svid_1 = getBits(dword_1, 6);
    signed delta_root_a_1 = getBits(dword_1, 2); // 11 left


    uint32_t dword_2 = getDataWord();
    signed delta_root_a_2 = getBits(dword_2, 11);
    unsigned eccentricity = getBits(dword_2, 11);
    signed perigee_1 = getBits(dword_2, 10); // 6 left


    uint32_t dword_3 = getDataWord();
    signed perigee_2 = getBits(dword_3, 6);
    signed diff_ia_na = getBits(dword_3, 11);
    signed longitude_1 = getBits(dword_3, 15); // 1 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed longitude_2 = getBits(dword_data, 1);
    signed roc_ra = getBits(dword_data, 11);
    signed mean_anomaly = getBits(dword_data, 16);
    unsigned reserved = getBits(dword_data, 6);


    word_type_7.issue_of_data = issue_of_data;
    word_type_7.week_num = week_num;
    word_type_7.ref_time = ref_time;
    word_type_7.svid_1 = svid_1;
    word_type_7.delta_root_a = concatenateBits(delta_root_a_1, delta_root_a_2, 2, 11);
    word_type_7.eccentricity = eccentricity;
    word_type_7.perigee = concatenateBits(perigee_1, perigee_2, 10, 6);
    word_type_7.diff_ia_na = diff_ia_na;
    word_type_7.longitude = concatenateBits(longitude_1, longitude_2, 15, 1);
    word_type_7.roc_ra = roc_ra;
    word_type_7.mean_anomaly = mean_anomaly;
    word_type_7.reserved = reserved;


    nav_data[svId_-1].add(word_type_7, svId_, sigId_);


    return true;
  }


  else if (word_type_ == ALMANAC_2)  // Word Type 8
  {
    unsigned issue_of_data = getBits(dword_1, 4);
    signed clock_corr_bias = getBits(dword_1, 16);
    signed clock_corr_linear_1 = getBits(dword_1, 4); // 9 left


    uint32_t dword_2 = getDataWord();
    signed clock_corr_linear_2 = getBits(dword_2, 9);
    unsigned sig_health_e5b = getBits(dword_2, 2);
    unsigned sig_health_e1 = getBits(dword_2, 2);
    unsigned svid_2 = getBits(dword_2, 6);
    signed delta_root_a = getBits(dword_2, 13);


    uint32_t dword_3 = getDataWord();
    unsigned eccentricity = getBits(dword_3, 11);
    signed perigee = getBits(dword_3, 16);
    signed diff_ia_na_1 = getBits(dword_3, 5); // 6 left


    uint64_t dword_4 = getDataWord();
    uint64_t  dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed diff_ia_na_2 = getBits(dword_data, 6);
    signed longitude = getBits(dword_data, 16);
    signed roc_ra = getBits(dword_data, 11);
    unsigned spare = getBits(dword_data, 1);


    word_type_8.issue_of_data = issue_of_data;
    word_type_8.clock_corr_bias = clock_corr_bias;
    word_type_8.clock_corr_linear = concatenateBits(clock_corr_linear_1, clock_corr_linear_2, 4, 9);
    word_type_8.sig_health_e5b = sig_health_e5b;
    word_type_8.sig_health_e1 = sig_health_e1;
    word_type_8.svid_2 = svid_2;
    word_type_8.delta_root_a = delta_root_a;
    word_type_8.eccentricity = eccentricity;
    word_type_8.perigee = perigee;
    word_type_8.diff_ia_na = concatenateBits(diff_ia_na_1, diff_ia_na_2, 5, 6);
    word_type_8.longitude = longitude;
    word_type_8.roc_ra = roc_ra;
    word_type_8.spare = spare;


    nav_data[svId_-1].add(word_type_8, svId_, sigId_);


    return true;
  }


  else if (word_type_ == ALMANAC_3) // Word Type 9
  {
    unsigned issue_of_data = getBits(dword_1, 4);
    unsigned week_num = getBits(dword_1, 2);
    unsigned ref_time = getBits(dword_1, 10);
    signed mean_anomaly_1 = getBits(dword_1, 8); // 8 left


    uint32_t dword_2 = getDataWord();
    signed mean_anomaly_2 = getBits(dword_2, 8);
    signed clock_corr_bias = getBits(dword_2, 16);
    signed clock_corr_linear_1 = getBits(dword_2, 8); // 5 left


    uint32_t dword_3 = getDataWord();
    signed clock_corr_linear_2 = getBits(dword_3, 5);
    unsigned sig_health_e5b = getBits(dword_3, 2);
    unsigned sig_health_e1 = getBits(dword_3, 2);
    unsigned svid_3 = getBits(dword_3, 6);
    signed delta_root_a = getBits(dword_3, 13);
    unsigned eccentricity_1 = getBits(dword_3, 4); // 7 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned eccentricity_2 = getBits(dword_data, 7);
    signed perigee = getBits(dword_data, 16);
    signed diff_ia_na = getBits(dword_data, 11);


    word_type_9.issue_of_data = issue_of_data;
    word_type_9.week_num = week_num;
    word_type_9.ref_time = ref_time;
    word_type_9.mean_anomaly = concatenateBits(mean_anomaly_1, mean_anomaly_2, 8, 8);
    word_type_9.clock_corr_bias = clock_corr_bias;
    word_type_9.clock_corr_linear = concatenateBits(clock_corr_linear_1, clock_corr_linear_2, 8, 5);
    word_type_9.sig_health_e5b = sig_health_e5b;
    word_type_9.sig_health_e1 = sig_health_e1;
    word_type_9.svid_3 = svid_3;
    word_type_9.delta_root_a = delta_root_a;
    word_type_9.eccentricity = concatenateBits(eccentricity_1, eccentricity_2, 4, 7);
    word_type_9.perigee = perigee;
    word_type_9.diff_ia_na = diff_ia_na;


    nav_data[svId_-1].add(word_type_9, svId_, sigId_);


    return true;
  }


  else if (word_type_ == ALMANAC_4) // Word Type 10
  {
    unsigned issue_of_data = getBits(dword_1, 4);
    signed longitude = getBits(dword_1, 16);
    signed roc_ra_1 = getBits(dword_1, 4); // 7 left


    uint32_t dword_2 = getDataWord();
    signed roc_ra_2 = getBits(dword_2, 7);
    signed mean_anomaly = getBits(dword_2, 16);
    signed clock_corr_bias_1 = getBits(dword_2, 9); // 7 left


    uint32_t dword_3 = getDataWord();
    signed clock_corr_bias_2 = getBits(dword_3, 7);
    signed clock_corr_linear = getBits(dword_3, 13);
    unsigned sig_health_e5b = getBits(dword_3, 2);
    unsigned sig_health_e1 = getBits(dword_3, 2);
    signed const_term_offset_1 = getBits(dword_3, 8); // 8 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed const_term_offset_2 = getBits(dword_data, 8);
    signed roc_offset = getBits(dword_data, 12);
    unsigned ref_time = getBits(dword_data, 8);
    unsigned week_num = getBits(dword_data, 6);


    word_type_10.issue_of_data = issue_of_data;
    word_type_10.longitude = longitude;
    word_type_10.roc_ra = concatenateBits(roc_ra_1, roc_ra_2, 4, 7);
    word_type_10.mean_anomaly = mean_anomaly;
    word_type_10.clock_corr_bias = concatenateBits(clock_corr_bias_1, clock_corr_bias_2, 9, 7);
    word_type_10.clock_corr_linear = clock_corr_linear;
    word_type_10.sig_health_e5b = sig_health_e5b;
    word_type_10.sig_health_e1 = sig_health_e1;
    word_type_10.const_term_offset = concatenateBits(const_term_offset_1, const_term_offset_2, 8, 8);
    word_type_10.roc_offset = roc_offset;
    word_type_10.ref_time = ref_time;
    word_type_10.week_num = week_num;


    nav_data[svId_-1].add(word_type_10, svId_, sigId_);


    return true;
  }


  else if (word_type_ == REDUCED_CED) // Word Type 16
  {
    signed delta_rced_smajor = getBits(dword_1, 5);
    signed eccentricity_rced_x = getBits(dword_1, 13);
    signed eccentricity_rced_y_1 = getBits(dword_1, 6); // 7 left


    uint32_t dword_2 = getDataWord();
    signed eccentricity_rced_y_2 = getBits(dword_2, 7);
    signed delta_rced_inclination = getBits(dword_2, 17);
    signed rced_longitude_1 = getBits(dword_2, 8); // 15 left


    uint32_t dword_3 = getDataWord();
    signed rced_longitude_2 = getBits(dword_3, 15);
    signed lambda_rced_1 = getBits(dword_3, 17); // 6 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    signed lambda_rced_2 = getBits(dword_data, 6);
    signed rced_clock_corr_bias = getBits(dword_data, 22);
    signed rced_clock_corr_drift = getBits(dword_data, 6);


    word_type_16.delta_rced_smajor = delta_rced_smajor;
    word_type_16.eccentricity_rced_x = eccentricity_rced_x;
    word_type_16.eccentricity_rced_y = concatenateBits(eccentricity_rced_y_1, eccentricity_rced_y_2, 6, 7);
    word_type_16.delta_rced_inclination = delta_rced_inclination;
    word_type_16.rced_longitude = concatenateBits(rced_longitude_1, rced_longitude_2, 8, 15);
    word_type_16.lambda_rced = concatenateBits(lambda_rced_1, lambda_rced_2, 17, 6);
    word_type_16.rced_clock_corr_bias = rced_clock_corr_bias;
    word_type_16.rced_clock_corr_drift = rced_clock_corr_drift;

    return true;
  }


  else if (word_type_ == FEC2) // Word Type 17, 18, 19, 20
  {
    unsigned fec2_1 = getBits(dword_1, 8);
    unsigned lsb = getBits(dword_1, 2);
    unsigned long long fec2_2_1 = getBits(dword_1, 14); // 50 left


    uint32_t dword_2 = getDataWord();
    unsigned long long fec2_2_2 = getBits(dword_2, 32); // 18 left


    uint32_t dword_3 = getDataWord();
    unsigned long long fec2_2_3 = getBits(dword_3, 18);
    unsigned long long fec2_3_1 = getBits(dword_3, 14); // 34 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned long long fec2_3_2 = getBits(dword_data, 34);

    fec2_2_2 = concatenateBits(fec2_2_2, fec2_2_3, 32, 18);

    word_type_17.fec2_1 = fec2_1;
    word_type_17.lsb = lsb;
    word_type_17.fec2_2 = concatenateBits(fec2_2_1, fec2_2_2, 14, 50);
    word_type_17.fec2_3 = concatenateBits(fec2_3_1, fec2_3_2, 14, 34);

    return true;
  }


  else if (word_type_ == SPARE) // Word Type 0
  {
    unsigned time = getBits(dword_1, 2);
    unsigned long long spare_1 = getBits(dword_1, 22); // 42 left


    uint32_t dword_2 = getDataWord();
    unsigned long long spare_2 = getBits(dword_2, 32); // 10 left


    uint32_t dword_3 = getDataWord();
    unsigned long long spare_3 = getBits(dword_3, 10);
    unsigned spare2_1 = getBits(dword_3, 22); // 2 left


    uint64_t dword_4 = getDataWord();
    uint64_t dword_5 = getDataWord();
    uint64_t dword_middle = concatenateBits(dword_4, dword_5, 32, 32);


    uint64_t dword_util = dword_middle;
    maskWordUtilMiddle(dword_util);
    word_util.tail = getBits(dword_util, 6);
    word_util.even_odd = getBits(dword_util, 1);
    word_util.page_type = getBits(dword_util, 1);

    if (word_util.tail != 0) { false_counter++; return false; }
    if (even_ == 0) { if (word_util.even_odd != 1) { false_counter++; return false; } }
    if (even_ == 1) { if (word_util.even_odd != 0) { false_counter++; return false; } }


    uint64_t dword_data = dword_middle;
    maskWordDataMiddle(dword_data);
    unsigned spare2_2 = getBits(dword_data, 2);
    unsigned week_num = getBits(dword_data, 12);
    unsigned time_of_week = getBits(dword_data, 20);

    spare_2 = concatenateBits(spare_2, spare_3, 32, 10);

    word_type_0.time = time;
    word_type_0.spare = concatenateBits(spare_1, spare_2, 22, 42);
    word_type_0.spare2 = concatenateBits(spare2_1, spare2_2, 22, 2);
    word_type_0.week_num = week_num;
    word_type_0.time_of_week = time_of_week;

    return true;
  }

  else if (word_type_ == DUMMY) 
    return true;

  else 
    return false;
  
}


uint32_t GalileoSolver::getDataWord() 
{
  char *buffer = new char[4];
  raw_data_.read(buffer, 4);

  uint32_t *dword_ref = reinterpret_cast<uint32_t *>(buffer);
  uint32_t dword = *dword_ref;

  delete[] buffer;

  return dword;
}


void GalileoSolver::maskWordUtilMiddle(uint64_t &dword_util) 
{
  dword_util = dword_util & MASK1_;
  dword_util = (dword_util << 18) | (dword_util << 26);
}


void GalileoSolver::maskWordDataMiddle(uint64_t &dword_data) 
{
  bitsize_ = 34;
  uint64_t dword_data1 = dword_data & MASK2_;
  uint64_t dword_data2 = dword_data & MASK3_;
  dword_data = (dword_data1) | (dword_data2 << 16);
}


void GalileoSolver::gnssCount(MessageDataHead &payload) 
{
  switch (payload.gnssId) 
  {
  case 0:
    if (msg_type_ == UBX_RXM_SFRBX)
      gps_num_sfrbx_++;
    else
      warn();
    break;

  case 1:
    if (msg_type_ == UBX_RXM_SFRBX)
      sbas_num_sfrbx_++;
    else
      warn();
    break;

  case 2:
    if (msg_type_ == UBX_RXM_SFRBX)
      galileo_num_sfrbx_++;
    else
      warn();
    break;

  case 3:
    if (msg_type_ == UBX_RXM_SFRBX)
      beidou_num_sfrbx_++;
    else
      warn();
    break;

  case 5:
    if (msg_type_ == UBX_RXM_SFRBX)
      qzss_num_sfrbx_++;
    else
      warn();
    break;

  case 6:
    if (msg_type_ == UBX_RXM_SFRBX)
      glonass_num_sfrbx_++;
    else
      warn();
    break;

  default:
    warn();
    break;
  }
}


void GalileoSolver::gnssCount(SignalInformation &payload) 
{
  switch (payload.gnssId) 
  {
  case 0:
    if (msg_type_ == UBX_NAV_SIG)
      gps_num_navsig_++;
    else
      warn();
    break;

  case 1:
    if (msg_type_ == UBX_NAV_SIG)
      sbas_num_navsig_++;
    else
      warn();
    break;

  case 2:
    if (msg_type_ == UBX_NAV_SIG)
      galileo_num_navsig_++;
    else
      warn();
    break;

  case 3:
    if (msg_type_ == UBX_NAV_SIG)
      beidou_num_navsig_++;
    else
      warn();
    break;

  case 5:
    if (msg_type_ == UBX_NAV_SIG)
      qzss_num_navsig_++;
    else
      warn();
    break;

  case 6:
    if (msg_type_ == UBX_NAV_SIG)
      glonass_num_navsig_++;
    else
      warn();
    break;

  default:
    warn();
    break;
  }
}


void GalileoSolver::classifySvid() 
{
  if (payload_sfrbx_head.svId == 1) ++svid1_counter;
  else if (payload_sfrbx_head.svId == 2) ++svid2_counter;
  else if (payload_sfrbx_head.svId == 3) ++svid3_counter;
  else if (payload_sfrbx_head.svId == 4) ++svid4_counter;
  else if (payload_sfrbx_head.svId == 5) ++svid5_counter;
  else if (payload_sfrbx_head.svId == 6) ++svid6_counter;
  else if (payload_sfrbx_head.svId == 7) ++svid7_counter;
  else if (payload_sfrbx_head.svId == 8) ++svid8_counter;
  else if (payload_sfrbx_head.svId == 9) ++svid9_counter;
  else if (payload_sfrbx_head.svId == 10) ++svid10_counter;
  else if (payload_sfrbx_head.svId == 11) ++svid11_counter;
  else if (payload_sfrbx_head.svId == 12) ++svid12_counter;
  else if (payload_sfrbx_head.svId == 13) ++svid13_counter;
  else if (payload_sfrbx_head.svId == 14) ++svid14_counter;
  else if (payload_sfrbx_head.svId == 15) ++svid15_counter;
  else if (payload_sfrbx_head.svId == 16) ++svid16_counter;
  else if (payload_sfrbx_head.svId == 17) ++svid17_counter;
  else if (payload_sfrbx_head.svId == 18) ++svid18_counter;
  else if (payload_sfrbx_head.svId == 19) ++svid19_counter;
  else if (payload_sfrbx_head.svId == 20) ++svid20_counter;
  else if (payload_sfrbx_head.svId == 21) ++svid21_counter;
  else if (payload_sfrbx_head.svId == 22) ++svid22_counter;
  else if (payload_sfrbx_head.svId == 23) ++svid23_counter;
  else if (payload_sfrbx_head.svId == 24) ++svid24_counter;
  else if (payload_sfrbx_head.svId == 25) ++svid25_counter;
  else if (payload_sfrbx_head.svId == 26) ++svid26_counter;
  else if (payload_sfrbx_head.svId == 27) ++svid27_counter;
  else if (payload_sfrbx_head.svId == 28) ++svid28_counter;
  else if (payload_sfrbx_head.svId == 29) ++svid29_counter;
  else if (payload_sfrbx_head.svId == 30) ++svid30_counter;
  else if (payload_sfrbx_head.svId == 31) ++svid31_counter;
  else if (payload_sfrbx_head.svId == 32) ++svid32_counter;
  else if (payload_sfrbx_head.svId == 33) ++svid33_counter;
  else if (payload_sfrbx_head.svId == 34) ++svid34_counter;
  else if (payload_sfrbx_head.svId == 35) ++svid35_counter;
  else if (payload_sfrbx_head.svId == 36) ++svid36_counter;
}


void GalileoSolver::log() const 
{
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

  std::cout << "\nSVID 1: " << svid1_counter
            << "\nSVID 2: " << svid2_counter
            << "\nSVID 3: " << svid3_counter
            << "\nSVID 4: " << svid4_counter
            << "\nSVID 5: " << svid5_counter
            << "\nSVID 6: " << svid6_counter
            << "\nSVID 7: " << svid7_counter
            << "\nSVID 8: " << svid8_counter
            << "\nSVID 9: " << svid9_counter
            << "\nSVID 10: " << svid10_counter
            << "\nSVID 11: " << svid11_counter
            << "\nSVID 12: " << svid12_counter
            << "\nSVID 13: " << svid13_counter
            << "\nSVID 14: " << svid14_counter
            << "\nSVID 15: " << svid15_counter
            << "\nSVID 16: " << svid16_counter
            << "\nSVID 17: " << svid17_counter
            << "\nSVID 18: " << svid18_counter
            << "\nSVID 19: " << svid19_counter
            << "\nSVID 20: " << svid20_counter
            << "\nSVID 21: " << svid21_counter
            << "\nSVID 22: " << svid22_counter
            << "\nSVID 23: " << svid23_counter
            << "\nSVID 24: " << svid24_counter
            << "\nSVID 25: " << svid25_counter
            << "\nSVID 26: " << svid26_counter
            << "\nSVID 27: " << svid27_counter
            << "\nSVID 28: " << svid28_counter
            << "\nSVID 29: " << svid29_counter
            << "\nSVID 30: " << svid30_counter
            << "\nSVID 31: " << svid31_counter
            << "\nSVID 32: " << svid32_counter
            << "\nSVID 33: " << svid33_counter
            << "\nSVID 34: " << svid34_counter
            << "\nSVID 35: " << svid35_counter
            << "\nSVID 36: " << svid36_counter
            << std::endl;

  
  std::cout << "\nWord Type 0: " << wordtype0_counter
            << "\nWord Type 1: " << wordtype1_counter
            << "\nWord Type 2: " << wordtype2_counter
            << "\nWord Type 3: " << wordtype3_counter
            << "\nWord Type 4: " << wordtype4_counter
            << "\nWord Type 5: " << wordtype5_counter
            << "\nWord Type 6: " << wordtype6_counter
            << "\nWord Type 7: " << wordtype7_counter
            << "\nWord Type 8: " << wordtype8_counter
            << "\nWord Type 9: " << wordtype9_counter
            << "\nWord Type 10: " << wordtype10_counter
            << "\nWord Type 16: " << wordtype16_counter
            << "\nWord Type 17: " << wordtype17_counter
            << "\nWord Type 63: " << wordtype63_counter
            << std::endl;


  std::cout << "\nCounter: " << counter << std::endl;
  std::cout << "True: " << true_counter << std::endl;
  std::cout << "False: " << false_counter << std::endl;
}


void GalileoSolver::warn() const { std::cout << "WARNING!!!" << std::endl; }


std::ofstream nav_data_file_("../data/output_navdata.txt");


bool NavigationData::flag1_ = false;
bool NavigationData::flag2_ = false;
bool NavigationData::flag3_ = false;
bool NavigationData::flag4_ = false;


void NavigationData::checkFull() 
{
  if (flag1_ && flag2_ && flag3_ && !flag4_) 
  {
    writeHeader();
    flag4_ = true;
  }

  if (clock_bias_ != INIT && clock_drift_ != INIT && clock_drift_rate_ != INIT && issue_of_data_ != INIT &&
      crs_ != INIT && delta_n_ != INIT && mean_anomaly_ != INIT && cuc_ != INIT &&
      eccentricity_ != INIT && cus_ != INIT && semi_major_root_ != INIT && ref_time_ != INIT &&
      cic_ != INIT && omega0_ != INIT && cis_ != INIT && inclination_angle_ != INIT &&
      crc_ != INIT && omega_ != INIT && omega_dot_ != INIT && roc_inclination_angle_ != INIT &&
      sisa_ != INIT && bgd1_ != INIT && bgd2_ != INIT) 
  {
    if (prev_toe_ != ref_time_) { write(); prev_toe_ = ref_time_; }
    reset();
  }
}


void NavigationData::reset() 
{
  svId_ = 0;
  epoch_ = 0;
  clock_bias_ = INIT;
  clock_drift_ = INIT;
  clock_drift_rate_ = INIT;
  issue_of_data_ = INIT;
  crs_ = INIT;
  delta_n_ = INIT;
  mean_anomaly_ = INIT;
  cuc_ = INIT;
  eccentricity_ = INIT;
  cus_ = INIT;
  semi_major_root_ = INIT;
  ref_time_ = INIT;
  cic_ = INIT;
  omega0_ = INIT;
  cis_ = INIT;
  inclination_angle_ = INIT;
  crc_ = INIT;
  omega_ = INIT;
  omega_dot_ = INIT;
  roc_inclination_angle_ = INIT;
  week_num_ = 0;
  sisa_ = INIT;
  sig_health_validity_ = INIT;
  bgd1_ = INIT;
  bgd2_ = INIT;
}


void NavigationData::resetAlmanacE5()
{
  alm_e5_issue_of_data_ = INIT;
  alm_e5_week_num_ = 0;
  alm_e5_ref_time_ = 0;
  alm_e5_svid_ = 0;
  alm_e5_delta_root_a_ = INIT;
  alm_e5_eccentricity_ = INIT;
  alm_e5_perigee_ = INIT;
  alm_e5_diff_ia_na_ = INIT;
  alm_e5_longitude_ = INIT;
  alm_e5_roc_ra_ = INIT;
  alm_e5_mean_anomaly_ = INIT;
  alm_e5_clock_corr_bias_ = INIT;
  alm_e5_clock_corr_linear_ = INIT;
  alm_e5_sig_health_e5b_ = 0;
  alm_e5_sig_health_e1_ = 0;
}


void NavigationData::resetAlmanacE1()
{
  alm_e1_issue_of_data_ = INIT;
  alm_e1_week_num_ = 0;
  alm_e1_ref_time_ = 0;
  alm_e1_svid_ = 0;
  alm_e1_delta_root_a_ = INIT;
  alm_e1_eccentricity_ = INIT;
  alm_e1_perigee_ = INIT;
  alm_e1_diff_ia_na_ = INIT;
  alm_e1_longitude_ = INIT;
  alm_e1_roc_ra_ = INIT;
  alm_e1_mean_anomaly_ = INIT;
  alm_e1_clock_corr_bias_ = INIT;
  alm_e1_clock_corr_linear_ = INIT;
  alm_e1_sig_health_e5b_ = 0;
  alm_e1_sig_health_e1_ = 0;
}


void NavigationData::write() 
{
  std::cout.precision(12);

  std::cout << "\nE" << svId_ << std::fixed << "\t" << epoch_ << " " << (int)floor((epoch_ % 86400) / 3600) << " " << ((epoch_ % 3600) % 3600) / 60 
            << std::scientific << "\t" << clock_bias_  << "\t" << clock_drift_ << "\t" << clock_drift_rate_ << "\n";

  std::cout << "  \t" << issue_of_data_ << "\t" << crs_ 
                 << "\t" << delta_n_ << "\t" << mean_anomaly_ << "\n";

  std::cout << "  \t" << cuc_ << "\t" << eccentricity_ 
                 << "\t" << cus_ << "\t" << semi_major_root_ << "\n";

  std::cout << "  \t" << ref_time_ << "\t" << cic_ 
                 << "\t" << omega0_ << "\t" << cis_ << "\n";

  std::cout << "  \t" << inclination_angle_ << "\t" << crc_ 
                 << "\t" << omega_ << "\t" << omega_dot_ << "\n";
            
  std::cout << "  \t" << roc_inclination_angle_ << "\t" << "\t"
                 << "  \t" << week_num_ << "\t" << double(0) << "\n";

  std::cout << "  \t" << sisa_ << "\t" << sig_health_validity_
                 << "\t" << bgd1_ << "\t" << bgd2_ << "\n";  

  //usleep(500000);

  nav_data_file_ << "\nE" << svId_ << std::fixed << "\t" << epoch_ << " " << (int)floor((epoch_ % 86400) / 3600) << " " << ((epoch_ % 3600) % 3600) / 60 << "\t" 
                 << std::scientific << std::setprecision(12) << clock_bias_ << "\t" << clock_drift_ << "\t" << clock_drift_rate_ << "\n";

  nav_data_file_ << "  \t" << issue_of_data_ << "\t" << crs_ 
                 << "\t" << delta_n_ << "\t" << mean_anomaly_ << "\n";

  nav_data_file_ << "  \t" << cuc_ << "\t" << eccentricity_ 
                 << "\t" << cus_ << "\t" << semi_major_root_ << "\n";

  nav_data_file_ << "  \t" << ref_time_ << "\t" << cic_ 
                 << "\t" << omega0_ << "\t" << cis_ << "\n";

  nav_data_file_ << "  \t" << inclination_angle_ << "\t" << crc_ 
                 << "\t" << omega_ << "\t" << omega_dot_ << "\n";
            
  nav_data_file_ << "  \t" << roc_inclination_angle_ << "\t" << "\t"
                 << "  \t" << week_num_ << "\t" << double(0) << "\n";

  nav_data_file_ << "  \t" << sisa_ << "\t" << sig_health_validity_
                 << "\t" << bgd1_ << "\t" << bgd2_ << "\n";

}


void NavigationData::writeHeader() 
{
  nav_data_file_ << "\n\n";
  nav_data_file_ << "\t\tHEADER\n";
  nav_data_file_ << "GAL\t" << gal_ai0_ << "\t" << gal_ai1_ << "\t" << gal_ai2_ << "\tIONOSPHERIC CORR\n";
  nav_data_file_ << "GAUT\t" << gaut_a0_ << "\t" << gaut_a1_ << "\t" << gaut_tow_ << "\t" << gaut_week_ << "\tTIME SYSTEM CORR\n";
  nav_data_file_ << "GPGA\t" << gpga_a0g_ << "\t" << gpga_a1g_ << "\t" << gpga_tow_ << "\t" << gpga_week_ << "\tTIME SYSTEM CORR\n\n";

  std::cout.precision(12);

  std::cout << "\n\n";
  std::cout << "\t\tHEADER\n";
  std::cout << "GAL\t" << std::scientific << gal_ai0_ << "\t" << gal_ai1_ << "\t" << gal_ai2_ << "\tIONOSPHERIC CORR\n";
  std::cout << "GAUT\t" << gaut_a0_ << "\t" << gaut_a1_ << "\t" << std::fixed << gaut_tow_ << "\t" << gaut_week_ << "\tTIME SYSTEM CORR\n";
  std::cout << "GPGA\t" << std::scientific << gpga_a0g_ << "\t" << gpga_a1g_ << "\t" << std::fixed << gpga_tow_ << "\t" << gpga_week_ << "\tTIME SYSTEM CORR\n\n";

  //std::cin.get();
}


void NavigationData::writeAlmanac(uint8_t sigId)
{
  if (sigId == 5)
  {
    std::cout << "Signal: " << (unsigned int)sigId << std::endl;

    std::cout << "SV ID: " << alm_e5_svid_ << std::endl;
    std::cout << "Issue of data: " << alm_e5_issue_of_data_ << std::endl;
    std::cout << "Week Num: " << alm_e5_week_num_ << std::endl;
    std::cout << "TOW: " << alm_e5_ref_time_ << std::endl;
    std::cout << "Delta root a: " << alm_e5_delta_root_a_ << std::endl;
    std::cout << "Eccentricity: " << alm_e5_eccentricity_ << std::endl;
    std::cout << "Perigee: " << alm_e5_perigee_ << std::endl;
    std::cout << "Diff IA NA: " << alm_e5_diff_ia_na_ << std::endl;
    std::cout << "Longitude: " << alm_e5_longitude_ << std::endl;
    std::cout << "Roc Ra: " << alm_e5_roc_ra_ << std::endl;
    std::cout << "Mean Anomaly: " << alm_e5_mean_anomaly_ << std::endl;
    std::cout << "Clock Corr Bias: " << alm_e5_clock_corr_bias_ << std::endl;
    std::cout << "Clock COrr Linear: " << alm_e5_clock_corr_linear_ << std::endl;
    std::cout << "Sig health e5b: " << alm_e5_sig_health_e5b_ << std::endl;
    std::cout << "Sig health e1: " << alm_e5_sig_health_e1_ << std::endl;
    std::cout << "\n\n\n";
  }

  else if (sigId == 1)
  {
    std::cout << "Signal: " << (unsigned int)sigId << std::endl;

    std::cout << "SV ID: " << alm_e1_svid_ << std::endl;
    std::cout << "Issue of data: " << alm_e1_issue_of_data_ << std::endl;
    std::cout << "Week Num: " << alm_e1_week_num_ << std::endl;
    std::cout << "TOW: " << alm_e1_ref_time_ << std::endl;
    std::cout << "Delta root a: " << alm_e1_delta_root_a_ << std::endl;
    std::cout << "Eccentricity: " << alm_e1_eccentricity_ << std::endl;
    std::cout << "Perigee: " << alm_e1_perigee_ << std::endl;
    std::cout << "Diff IA NA: " << alm_e1_diff_ia_na_ << std::endl;
    std::cout << "Longitude: " << alm_e1_longitude_ << std::endl;
    std::cout << "Roc Ra: " << alm_e1_roc_ra_ << std::endl;
    std::cout << "Mean Anomaly: " << alm_e1_mean_anomaly_ << std::endl;
    std::cout << "Clock Corr Bias: " << alm_e1_clock_corr_bias_ << std::endl;
    std::cout << "Clock COrr Linear: " << alm_e1_clock_corr_linear_ << std::endl;
    std::cout << "Sig health e5b: " << alm_e1_sig_health_e5b_ << std::endl;
    std::cout << "Sig health e1: " << alm_e1_sig_health_e1_ << std::endl;
    std::cout << "\n\n\n";
  }

  // std::cin.get();
  
}