#ifndef GALILEO_GALILEO_PARSER_H
#define GALILEO_GALILEO_PARSER_H

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

class GalileoParser {
private:
  const std::string file_;
  std::ifstream raw_data_;

  uint8_t byte_;

  const uint8_t SYNC_HEADER_1_ = 0xb5;
  const uint8_t SYNC_HEADER_2_ = 0x62;

  bool sync_lock_1_ = false;
  bool sync_lock_2_ = false;

  unsigned int counter = 0;
  unsigned int true_counter = 0;
  unsigned int false_counter = 0;

  enum MessageType { UBX_RXM_SFRBX, UBX_NAV_SIG, NOT_DEFINED } msg_type_;

  #pragma pack(1)
  struct MessageData {
    uint8_t message_class;
    uint8_t message_id;
    uint16_t length;
  } msg_data;

  struct NavigationDataHead {
    uint8_t gnssId;
    uint8_t svId;
    uint8_t reserved0;
    uint8_t freqId;
    uint8_t numWords;
    uint8_t chn;
    uint8_t version;
    uint8_t reserved1;
  } payload_sfrbx_head;
  
  struct NavigationDataWordHead {
    unsigned short even_odd : 1;
    unsigned short page_type : 1;
    unsigned short word_type : 6;
  } payload_data_word_head;


  /**
   * @brief Word Type 1: Ephemeris (1/4)
   * 
   * @param reference_time t0e Ephemeris reference time
   * @param mean_anomaly M0 Mean anomaly at reference time
   * @param eccentricity e Eccentricity
   * @param root_semi_major_axis A1/2 Square root of the semi-major axis
   * 
   */
  struct WordType1 {
    unsigned issue_of_data : 10;
    unsigned reference_time : 14;
    signed mean_anomaly : 32; 
    unsigned eccentricity : 32;
    unsigned root_semi_major_axis : 32; 
    unsigned reserved : 2;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 2: Ephemeris (2/4)
   * 
   * @param longitude Ω0 Longitude of ascending node of
   *                  orbital plane at weekly epoch
   * @param inclination_angle i0 Inclination angle at reference time
   * @param perigee ω Argument of perigee
   * @param ia_rate_of_change i^dot Rate of change of inclination angle
   * 
   */
  struct WordType2 {
    unsigned issue_of_data : 10;
    signed longitude : 32;
    signed inclination_angle : 32;
    signed perigee : 32;
    signed ia_rate_of_change : 14;
    unsigned reserved : 2;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 3: Ephemeris (3/4) and SISA
   * 
   * @param ra_rate_of_change Ω^dot Rate of change of right ascension
   * @param mean_motion_difference ∆n Mean motion difference from computed value
   * @param C_uc Cuc Amplitude of the cosine harmonic correction 
   *             term to the argument of latitude
   * @param C_us Cus Amplitude of the sine harmonic correction term 
   *             to the argument of latitude
   * @param C_rc Crc Amplitude of the cosine harmonic correction 
   *             term to the orbit radius
   * @param C_rs Crs Amplitude of the sine harmonic correction term 
   *             to the orbit radius
   * @param sisa SISA Signal-In-Space Accuracy
   * 
   */
  struct WordType3 {
    unsigned issue_of_data : 10;
    signed ra_rate_of_change : 24;
    signed mean_motion_difference : 16;
    signed C_uc : 16;
    signed C_us : 16;
    signed C_rc : 16;
    signed C_rs : 16;
    unsigned sisa : 8;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 4: SVID, Ephemeris (4/4),
   *        and Clock correction parameters 
   * 
   * @param svid Space Vehicle Identifier
   * @param C_ic Cic Amplitude of the cosine harmonic correction 
   *             term to the angle of inclination
   * @param C_is Cis Amplitude of the sine harmonic correction term 
   *             to the angle of inclination
   * @param reference t0c Clock correction data reference Time of Week
   * @param clock_bias_corr af0 SV clock bias correction coefficient
   * @param clock_drift_corr af1 SV clock drift correction coefficient
   * @param clock_drift_rate_corr af2 SV clock drift rate correction coefficient
   * 
   */
  struct WordType4 {
    unsigned issue_of_data : 10;
    unsigned svid : 6;
    signed C_ic : 16;
    signed C_is : 16;
    unsigned reference : 14;
    signed clock_bias_corr : 31;
    signed clock_drift_corr : 21;
    signed clock_drift_rate_corr : 6;
    unsigned spare : 2;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 5: Ionospheric correction, BGD, 
   * signal health and data validity status and GST
   * 
   * @param effionl_0 ai0 Effective Ionisation Level 1st order parameter
   * @param effionl_1 ai1 Effective Ionisation Level 2nd order parameter
   * @param effionl_2 ai2 Effective Ionisation Level 3rd order parameter
   * @param region1 SF1 Ionospheric Disturbance Flag for region 1
   * @param region2 SF2 Ionospheric Disturbance Flag for region 2
   * @param region3 SF3 Ionospheric Disturbance Flag for region 3
   * @param region4 SF4 Ionospheric Disturbance Flag for region 4
   * @param reigon5 SF5 Ionospheric Disturbance Flag for region 5
   * @param bgd_1 BGD(E1,E5a) E1-E5a Broadcast Group Delay
   * @param bgd_2 BGD(E1,E5b) E1-E5b Broadcast Group Delay
   * @param sig_health_e5b E5bHS E5b Signal Health Status
   * @param sig_health_e1 E1-BHS E1-B/C Signal Health Status
   * @param data_validity_e5b E5bDVS E5b Data Validity Status
   * @param data_validity_e1 E1-BDVS E1-B Data Validity Status
   * @param week_num WN Week Number
   * @param time_of_week TOW Time of Week
   * 
   */
  struct WordType5 {
    unsigned effionl_0 : 11;
    signed effionl_1 : 11;
    signed effionl_2 : 14;
    unsigned region1 : 1;
    unsigned region2 : 1;
    unsigned region3 : 1;
    unsigned region4 : 1;
    unsigned region5 : 1;
    signed bgd_1 : 10;
    signed bgd_2 : 10;
    unsigned sig_health_e5b : 2;
    unsigned sig_health_e1 : 2;
    unsigned data_validity_e5b : 1;
    unsigned data_validity_e1 : 1;
    unsigned week_num : 12;
    unsigned time_of_week : 20;
    unsigned spare : 23;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 6: GST-UTC conversion parameters
   * 
   * @param A0 A0 Constant term of polynomial
   * @param A1 A1 1st order term of polynomial
   * @param ls_count_before ΔtLS Leap Second count before leap
   *                          second adjustment
   * @param utc_reference_tow t0t UTC data reference Time of Week
   * @param utc_reference_week WN0t UTC data reference Week Number
   * @param WN_lsf WNLSF Week Number of leap second adjustment
   * @param day_num DN Day Number at the end of which a leap second 
   *                adjustment becomes effective. The value range of 
   *                DN is from 1 (= Sunday) to 7 (= Saturday).
   * @param ls_count_after ΔtLSF Leap Second count after leap second adjustment
   * @param time_of_week TOW Time of Week
   * 
   */
  struct WordType6 {
    signed A0 : 32;
    signed A1 : 24;
    signed ls_count_before : 8;
    unsigned utc_reference_tow : 8;
    unsigned utc_reference_week : 8;
    unsigned WN_lsf : 8;
    unsigned day_num : 3;
    signed ls_count_after : 8;
    unsigned time_of_week : 20;
    unsigned spare : 8;
    unsigned tail : 6;
  };


  /**
   * @brief Almanac for SVID1 (1/2), almanac reference time and 
   *        almanac reference week number
   * 
   * @param week_num WNa Almanac reference Week Number
   * @param ref_time t0a Almanac reference time
   * @param svid_1 SVID Satellite ID (1 constellation of 36 satellites)
   * @param delta_root_a Δ(A1/2) Difference between the square root of the 
   *                     semi-major axis and the square root of the 
   *                     nominal semi-major axis      
   * @param eccentricity Eccentricity 
   * @param perigee ω Argument of perigee
   * @param diff_ia_na δi Difference between the inclination angle at 
   *                   reference time and the nominal inclination 
   * @param longitude Ω0 Longitude of ascending node of orbital plane 
   *                  at weekly epoch
   * @param roc_ra Ω^dot Rate of change of right ascension
   * @param mean_anomaly M0 Satellite mean anomaly at reference time
   * 
   */
  struct WordType7 {
    unsigned issue_of_data : 4;
    unsigned week_num : 2;
    unsigned ref_time : 10;
    unsigned svid_1 : 6;
    signed delta_root_a : 13;
    unsigned eccentricity : 11;
    signed perigee : 16;
    signed diff_ia_na : 11;
    signed longitude : 16;
    signed roc_ra : 11;
    signed mean_anomaly : 16;
    unsigned reserved : 6;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 8: Almanac for SVID1 (2/2) and SVID2 (1/2))
   * 
   * @param clock_corr_bias af0 Satellite clock correction bias “truncated”
   * @param clock_corr_linear af1 Satellite clock correction linear “truncated”
   * @param sig_health_e5b E5bHS Satellite E5b signal health status 
   * @param sig_health_e1 E1-BHS Satellite E1-B/C signal health status
   * @param svid_2 SVID Satellite ID (1 constellation of 36 satellites)
   * @param delta_root_a Δ(A1/2) Difference between the square root of the 
   *                   semi-major axis and the square root of the 
   *                   nominal semi-major axis
   * @param eccentricity e Eccentricity
   * @param perigee ω Argument of perigee
   * @param diff_ia_na δi Difference between the inclination angle at 
   *                   reference time and the nominal inclination
   * @param longitude Ω0 Longitude of ascending node of orbital plane 
   *                  at weekly epoch   
   * @param roc_ra Ω^dot Rate of change of right ascension  
   * 
   */
  struct WordType8 {
    unsigned issue_of_data : 4;
    signed clock_corr_bias : 16;
    signed clock_corr_linear : 13;
    unsigned sig_health_e5b : 2;
    unsigned sig_health_e1 : 2;
    unsigned svid_2 : 6;
    signed delta_root_a : 13;
    unsigned eccentricity : 11;
    signed perigee : 16;
    signed diff_ia_na : 11;
    signed longitude : 16;
    signed roc_ra : 11;
    unsigned spare : 1;
    unsigned tail : 6;
  };


  /**
   * @brief Word Type 9: Almanac for SVID2 (2/2) and SVID3 (1/2))
   * 
   * @param week_num WNa Almanac reference Week Number
   * @param ref_time t0a Almanac reference time
   * @param mean_anomaly M0 Satellite mean anomaly at reference time
   * @param clock_corr_bias af0 Satellite clock correction bias “truncated”
   * @param clock_corr_linear af1 Satellite clock correction linear “truncated”
   * @param sig_health_e5b Satellite E5b signal health status
   * @param sig_health_e1 E1-BHS Satellite E1-B/C signal health status
   * @param svid_3 SVID Satellite ID (1 constellation of 36 satellites)
   * @param delta_root_a Δ(A1/2) Difference between the square root of the 
   *                     semi-major axis and the square root of the 
   *                     nominal semi-major axis
   * @param eccentricity e Eccentricity
   * @param perigee ω Argument of perigee
   * @param diff_ia_na δi Difference between the inclination angle at 
   *                   reference time and the nominal inclination
   * 
   */
  struct WordType9 {
    unsigned issue_of_data : 4;
    unsigned week_num : 2;
    unsigned ref_time : 10;
    signed mean_anomaly : 16;
    signed clock_corr_bias : 16;
    signed clock_corr_linear : 13;
    unsigned sig_health_e5b : 2;
    unsigned sig_health_e1 : 2;
    unsigned svid_3 : 6;
    signed delta_root_a : 13;
    unsigned eccentricity : 11;
    signed perigee : 16;
    signed diff_ia_na : 11;
    unsigned tail : 6;
  };



  struct SignalInformationHead {
    uint32_t iTOW;
    uint8_t version;
    uint8_t numSigs;
    uint16_t reserved0;
  } payload_navsig_head;

  struct SignalInformation {
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
  #pragma pack()

  enum WordType {
    SPARE,
    EPHEMERIS_1,
    EPHEMERIS_2,
    EPHEMERIS_3,
    EPHEMERIS_4__CLOCK_CORRECTION,
    IONOSPHERIC_CORRECTION__BGD__SIG_HEALTH__DVS__GST,
    GST_UTC_CONVERSION,
    ALMANAC_1,
    ALMANAC_2,
    ALMANAC_3,
    ALMANAC_4,
    REDUCED_CED = 16,
    FEC2, 
    DUMMY = 63
  }word_type_;

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
  void CheckSyncHeaders(uint8_t &byte_);
  void ParseInitialData(std::ifstream &raw_data_, MessageType &msg_type);
  bool ParsePayloadData(std::ifstream &raw_data_);
  void ParseDataWord(std::ifstream &raw_data_);
  bool DetermineWordType(NavigationDataWordHead &payload_data_word_head);
  void GnssCount(NavigationDataHead &payload);
  void GnssCount(SignalInformation &payload);
  void Log() const;
  void Warn() const;
};

#endif // GALILEO_GALILEO_PARSER_H