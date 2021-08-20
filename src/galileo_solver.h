#ifndef GALILEO_GALILEO_SOLVER_H
#define GALILEO_GALILEO_SOLVER_H

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cmath>
#include <bitset>
#include <cfloat>

#define INIT DBL_MAX


class NavigationData 
{
private:
  unsigned int svId_;

  unsigned int epoch_;
  double clock_bias_ = INIT;
  double clock_drift_ = INIT;
  double clock_drift_rate_ = INIT;
  double issue_of_data_ = INIT;
  double crs_ = INIT;
  double delta_n_ = INIT;
  double mean_anomaly_ = INIT;
  double cuc_ = INIT;
  double eccentricity_ = INIT;
  double cus_ = INIT;
  double semi_major_root_ = INIT;
  double ref_time_ = INIT;
  double cic_ = INIT;
  double omega0_ = INIT;
  double cis_ = INIT;
  double inclination_angle_ = INIT;
  double crc_ = INIT;
  double omega_ = INIT;
  double omega_dot_ = INIT;
  double roc_inclination_angle_ = INIT;
  unsigned int week_num_;
  double sisa_ = INIT;
  double sig_health_validity_ = INIT;
  double bgd1_ = INIT;
  double bgd2_ = INIT;

private:
  double gal_ai0_;
  double gal_ai1_;
  double gal_ai2_;
  double gaut_a0_;
  double gaut_a1_;
  unsigned gaut_tow_;
  unsigned gaut_week_;
  double gpga_a0g_;
  double gpga_a1g_;
  unsigned gpga_tow_;
  unsigned gpga_week_;

private: 
  bool flag1_ = false;
  bool flag2_ = false;
  bool flag3_ = false;
  bool flag4_ = false;

public:
  template <class T> void add(T word, unsigned int type, unsigned int svId);
  void reset();
  bool check_full(unsigned int type);
  void write();
  void write_header();
};


class GalileoSolver 
{
private:
  const std::string file_;
  std::ifstream raw_data_;

  NavigationData space_vehicle[36]{};

  uint8_t byte_;

  const uint8_t SYNC_HEADER_1_ = 0xb5;
  const uint8_t SYNC_HEADER_2_ = 0x62;

  bool sync_lock_1_ = false;
  bool sync_lock_2_ = false;

  unsigned int counter = 0;
  unsigned int true_counter = 0;
  unsigned int false_counter = 0;

  unsigned short even_;
  unsigned int pos_;

  const unsigned long long MASK1_ = 0x3F00C0000000;
  const unsigned long long MASK2_ = 0xFFFFC00000000000;
  const unsigned long long MASK3_ = 0x3FFFC000;

  enum MessageType { UBX_RXM_SFRBX, UBX_NAV_SIG, NOT_DEFINED } msg_type_;


#pragma pack(1)

  struct MessageHead 
  {
    uint8_t message_class;
    uint8_t message_id;
    uint16_t length;
  } msg_head;


  struct CheckSumParams 
  {
    uint8_t ck_a;
    uint8_t ck_b;
  } checksum;


  struct MessageDataHead 
  {
    uint8_t gnssId;
    uint8_t svId;
    uint8_t reserved0;
    uint8_t freqId;
    uint8_t numWords;
    uint8_t chn;
    uint8_t version;
    uint8_t reserved1;
  } payload_sfrbx_head;


  struct MessageDataWordHead 
  {
    unsigned short even_odd : 1;
    unsigned short page_type : 1;
    unsigned short word_type : 6;
  } payload_data_word_head;


  struct WordUtil 
  {
    unsigned short tail : 6;
    unsigned short even_odd : 1;
    unsigned short page_type : 1;
  } word_util;


public:

  /**
   * @brief Word Type 1: Ephemeris (1/4)
   *
   * @param reference_time t0e Ephemeris reference time
   * @param mean_anomaly M0 Mean anomaly at reference time
   * @param eccentricity e Eccentricity
   * @param root_semi_major_axis A1/2 Square root of the semi-major axis
   *
   */
  struct WordType1 
  {
    unsigned issue_of_data : 10;
    unsigned reference_time : 14;
    signed mean_anomaly : 32;
    unsigned eccentricity : 32;
    unsigned root_semi_major_axis : 32;
    unsigned reserved : 2;
  } word_type_1;


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
  struct WordType2 
  {
    unsigned issue_of_data : 10;
    signed longitude : 32;
    signed inclination_angle : 32;
    signed perigee : 32;
    signed ia_rate_of_change : 14;
    unsigned reserved : 2;
  } word_type_2;


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
  struct WordType3 
  {
    unsigned issue_of_data : 10;
    signed ra_rate_of_change : 24;
    signed mean_motion_difference : 16;
    signed C_uc : 16;
    signed C_us : 16;
    signed C_rc : 16;
    signed C_rs : 16;
    unsigned sisa : 8;
  } word_type_3;


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
  struct WordType4 
  {
    unsigned issue_of_data : 10;
    unsigned svid : 6;
    signed C_ic : 16;
    signed C_is : 16;
    unsigned reference : 14;
    signed clock_bias_corr : 31;
    signed clock_drift_corr : 21;
    signed clock_drift_rate_corr : 6;
    unsigned spare : 2;
  } word_type_4;


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
  struct WordType5 
  {
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
    unsigned sig_health_validity;
    unsigned week_num : 12;
    unsigned time_of_week : 20;
    unsigned spare : 23;
  } word_type_5;


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
  struct WordType6 
  {
    signed A0 : 32;
    signed A1 : 24;
    signed ls_count_before : 8;
    unsigned utc_reference_tow : 8;
    unsigned utc_reference_week : 8;
    unsigned WN_lsf : 8;
    unsigned day_num : 3;
    signed ls_count_after : 8;
    unsigned time_of_week : 20;
    unsigned spare : 3;
  } word_type_6;


  /**
   * @brief Almanac for SVID1 (1/2), almanac reference time and
   *        almanac reference week number
   *
   * @param week_num WNa Almanac reference Week Number
   * @param ref_time_ t0a Almanac reference time
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
  struct WordType7 
  {
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
  } word_type_7;


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
  struct WordType8 
  {
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
  } word_type_8;


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
  struct WordType9 
  {
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
  } word_type_9;


  /**
   * @brief Word Type 10: Almanac for SVID3 (2/2)
   *        and GST-GPS conversion parameters
   *
   * @param longitude Ω0 Longitude of ascending node of orbital plane
   *                  at weekly epoch
   * @param roc_ra Ω^dot Rate of change of right ascension
   * @param mean_anomaly M0 Satellite mean anomaly at reference time
   * @param clock_corr_bias af0 Satellite clock correction bias “truncated”
   * @param clock_corr_linear af1 Satellite clock correction linear “truncated”
   * @param sig_health_e5b E5bHS Satellite E5b signal health status
   * @param sig_health_e1 E1-BHS Satellite E1-B/C signal health status
   * @param const_term_offset A0G Constant term of the polynomial describing the
   * offset
   * @param roc_offset A1G Rate of change of the offset
   * @param ref_time t0G Reference time for GGTO data
   * @param week_num WN0G Week Number of GGTO reference
   *
   */
  struct WordType10 
  {
    unsigned issue_of_data : 4;
    signed longitude : 16;
    signed roc_ra : 11;
    signed mean_anomaly : 16;
    signed clock_corr_bias : 16;
    signed clock_corr_linear : 13;
    unsigned sig_health_e5b : 2;
    unsigned sig_health_e1 : 2;
    signed const_term_offset : 16;
    signed roc_offset : 12;
    unsigned ref_time : 8;
    unsigned week_num : 6;
  } word_type_10;


  /**
   * @brief Word Type 16: Reduced Clock and Ephemeris Data (CED) parameters
   *
   * @param delta_rced_smajor Difference between the Reduced CED semi-major axis
   *                          and the nominal semi-major
   * @param eccentricity_rced_x Reduced CED eccentricity vector component x
   * @param eccentricity_rced_y Reduced CED eccentricity vector component y
   * @param delta_rced_inclination Difference between the Reduced CED
   * inclination angle at reference time and the nominal inclination
   * @param rced_longitude Reduced CED longitude of ascending node at weekly
   * epoch
   * @param lambda_rced Reduced CED mean argument of latitude
   * @param rced_clock_corr_bias Reduced CED satellite clock bias correction
   * coefficient
   * @param rced_clock_corr_drift Reduced CED satellite clock drift correction
   * coefficient
   *
   *
   */
  struct WordType16 
  {
    signed delta_rced_smajor : 5;
    signed eccentricity_rced_x : 13;
    signed eccentricity_rced_y : 13;
    signed delta_rced_inclination : 17;
    signed rced_longitude : 23;
    signed lambda_rced : 23;
    signed rced_clock_corr_bias : 22;
    signed rced_clock_corr_drift : 6;
  } word_type_16;


  /**
   * @brief Word types 17, 18, 19, 20: FEC2 Reed-Solomon
   *        for Clock and Ephemeris Data (CED)
   *
   */
  struct WordType17 
  {
    unsigned fec2_1 : 8;
    unsigned lsb : 2;
    unsigned long long fec2_2 : 64;
    unsigned long long fec2_3 : 48; // 112
  } word_type_17;


  /**
   * @brief Word Type 0: I/NAV Spare Word
   *
   */
  struct WordType0 
  {
    unsigned time : 2;
    unsigned long long spare : 64; // 88
    unsigned spare2 : 24;
    unsigned week_num : 12;
    unsigned time_of_week : 20;
  } word_type_0;


  /**
   * @brief Word Type 63: Dummy Message
   *
   */
  struct WordType63 {};


  struct SignalInformationHead 
  {
    uint32_t iTOW;
    uint8_t version;
    uint8_t numSigs;
    uint16_t reserved0;
  } payload_navsig_head;


  struct SignalInformation 
  {
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


  enum WordType 
  {
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
  } word_type_;


private:
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

  unsigned int svid1_counter = 0;
  unsigned int svid2_counter = 0;
  unsigned int svid3_counter = 0;
  unsigned int svid4_counter = 0;
  unsigned int svid5_counter = 0;
  unsigned int svid6_counter = 0;
  unsigned int svid7_counter = 0;
  unsigned int svid8_counter = 0;
  unsigned int svid9_counter = 0;
  unsigned int svid10_counter = 0;
  unsigned int svid11_counter = 0;
  unsigned int svid12_counter = 0;
  unsigned int svid13_counter = 0;
  unsigned int svid14_counter = 0;
  unsigned int svid15_counter = 0;
  unsigned int svid16_counter = 0;
  unsigned int svid17_counter = 0;
  unsigned int svid18_counter = 0;
  unsigned int svid19_counter = 0;
  unsigned int svid20_counter = 0;
  unsigned int svid21_counter = 0;
  unsigned int svid22_counter = 0;
  unsigned int svid23_counter = 0;
  unsigned int svid24_counter = 0;
  unsigned int svid25_counter = 0;
  unsigned int svid26_counter = 0;
  unsigned int svid27_counter = 0;
  unsigned int svid28_counter = 0;
  unsigned int svid29_counter = 0;
  unsigned int svid30_counter = 0;
  unsigned int svid31_counter = 0;
  unsigned int svid32_counter = 0;
  unsigned int svid33_counter = 0;
  unsigned int svid34_counter = 0;
  unsigned int svid35_counter = 0;
  unsigned int svid36_counter = 0;

  unsigned int wordtype0_counter = 0;
  unsigned int wordtype1_counter = 0;
  unsigned int wordtype2_counter = 0;
  unsigned int wordtype3_counter = 0;
  unsigned int wordtype4_counter = 0;
  unsigned int wordtype5_counter = 0;
  unsigned int wordtype6_counter = 0;
  unsigned int wordtype7_counter = 0;
  unsigned int wordtype8_counter = 0;
  unsigned int wordtype9_counter = 0;
  unsigned int wordtype10_counter = 0;
  unsigned int wordtype16_counter = 0;
  unsigned int wordtype17_counter = 0;
  unsigned int wordtype63_counter = 0;


public:
  explicit GalileoSolver(const std::string &path);

  void Read();
  void CheckSyncHeaders(uint8_t &byte_);
  bool CheckSum(std::ifstream &raw_data_);
  void ParseInitialData(std::ifstream &raw_data_);
  bool ParsePayloadData(std::ifstream &raw_data_);
  bool ParseDataWord(std::ifstream &raw_data_, unsigned int dword);
  uint32_t GetDataWord();
  uint32_t GetWordMiddle();
  void MaskWordUtilMiddle(uint64_t& dword_util);
  void MaskWordDataMiddle(uint64_t& dword_data);
  bool DetermineWordType(MessageDataWordHead &payload_data_word_head);
  void GnssCount(MessageDataHead &payload);
  void GnssCount(SignalInformation &payload);
  void ClassifySvid();
  template <typename T> T GetBits(T x, int n);
  template <typename T> T ConcatenateBits(T data1, T data2, int size1, int size2);
  void Log() const;
  void Warn() const;
};


template <class T> 
inline void NavigationData::add(T word, unsigned int type, unsigned int svId) {}


template <> 
inline void NavigationData::add<GalileoSolver::WordType1>(GalileoSolver::WordType1 word, unsigned int type, unsigned int svId) 
{
  svId_ = svId;
  issue_of_data_ = word.issue_of_data;
  ref_time_ = word.reference_time * 60; // scale factor 60
  mean_anomaly_ = word.mean_anomaly * pow(2, -31) * M_PI; // scale factor  2e-31
  eccentricity_ = word.eccentricity * pow(2, -33); // scale factor 2e-33
  semi_major_root_ = word.root_semi_major_axis * pow(2, -19); // scale factor 2e-19
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType2>(GalileoSolver::WordType2 word, unsigned int type, unsigned int svId) 
{
  issue_of_data_ = word.issue_of_data; 
  omega0_ = word.longitude * pow(2, -31) * M_PI; // scale factor 2e-31 
  inclination_angle_ = word.inclination_angle * pow(2, -31) * M_PI; // scale factor 2e-31
  omega_ = word.perigee * pow(2, -31) * M_PI; // scale factor 2e-31 
  roc_inclination_angle_ = word.ia_rate_of_change * pow(2, -43) * M_PI; // scale factor 2e-43 
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType3>(GalileoSolver::WordType3 word, unsigned int type, unsigned int svId) 
{
  issue_of_data_ = word.issue_of_data; 
  omega_dot_ = word.ra_rate_of_change * pow(2, -43) * M_PI; // scale factor 2e-43
  delta_n_ = word.mean_motion_difference * pow(2, -43) * M_PI; // scale factor 2e-43 
  cuc_ = word.C_uc * pow(2, -29); // scale factor 2e-29 
  cus_ = word.C_us * pow(2, -29); // scale factor 2e-29 
  crc_ = word.C_rc * pow(2, -5); // scale factor 2e-5 
  crs_ = word.C_rs * pow(2, -5); // scale factor 2e-5 
  sisa_ = word.sisa;
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType4>(GalileoSolver::WordType4 word, unsigned int type, unsigned int svId) // svid not included
{
  issue_of_data_ = word.issue_of_data; 
  cic_ = word.C_ic * pow(2, -29); // scale factor 2e-29
  cis_ = word.C_is * pow(2, -29); // scale factor 2e-29
  epoch_ = word.reference * 60; // scale factor 60
  clock_bias_ = word.clock_bias_corr * pow(2, -34); // scale factor 2e-34
  clock_drift_ = word.clock_drift_corr * pow(2, -46); // scale factor 2e-46
  clock_drift_rate_ = word.clock_drift_rate_corr * pow(2, -59); // scale factor 2e-59
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType5>(GalileoSolver::WordType5 word, unsigned int type, unsigned int svId) 
{
  if (!flag1_) 
  {
    gal_ai0_ = word.effionl_0 * pow(2, -2);
    gal_ai1_ = word.effionl_1 * pow(2, -8);
    gal_ai2_ = word.effionl_2 * pow(2, -15);
    flag1_ = true;
  }

  bgd1_ = word.bgd_1 * pow(2, -32); // scale factor 2e-32
  bgd2_ = word.bgd_2 * pow(2, -32); // scale factor 2e-32

  sig_health_validity_ = word.sig_health_validity;

  week_num_ = word.week_num; // scale factor 1
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType6>(GalileoSolver::WordType6 word, unsigned int type, unsigned int svId) 
{
  if (!flag2_) 
  {
    gaut_a0_ = word.A0 * pow(2, -30);
    gaut_a1_ = word.A1 * pow(2, -50);
    gaut_tow_ = word.utc_reference_tow * 3600;
    gaut_week_ = word.utc_reference_week;
    flag2_ = true;
  }
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType7>(GalileoSolver::WordType7 word, unsigned int type, unsigned int svId) 
{
  /*std::cout << "******************* ALMANAC ******************" << std::endl;
  std::cout << "SV: " << svId << "\tAlmanac of: " << word.svid_1 << "\tWord Type: " << type << std::endl; 
  std::cout << "IOD: " << word.issue_of_data << std::endl;
  std::cout << "Week num: " << word.week_num << std::endl;
  std::cout << "Ref time: " << word.ref_time << std::endl;
  std::cout << "delta_root_a:  " << word.delta_root_a << std::endl;
  std::cout << "eccentricity: " << word.eccentricity << std::endl;
  std::cout << "perigee: " << word.perigee << std::endl;*/
}


template <> 
inline void NavigationData::add<GalileoSolver::WordType8>(GalileoSolver::WordType8 word, unsigned int type, unsigned int svId) {}


template <> 
inline void NavigationData::add<GalileoSolver::WordType9>(GalileoSolver::WordType9 word, unsigned int type, unsigned int svId) {}


template <> 
inline void NavigationData::add<GalileoSolver::WordType10>(GalileoSolver::WordType10 word, unsigned int type, unsigned int svId) 
{
  if (!flag3_) 
  {
    gpga_a0g_ = word.const_term_offset * pow(2, -35);
    gpga_a1g_ = word.roc_offset * pow(2, -51);
    gpga_tow_ = word.ref_time * 3600;
    gpga_week_ = word.week_num;
    flag3_ = true;
  }
}


#endif // GALILEO_GALILEO_SOLVER_H