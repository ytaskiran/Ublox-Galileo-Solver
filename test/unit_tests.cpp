#include "galileo_solver.h"
#include <vector>
#include "gtest/gtest.h"

struct GalileoSolverTest : testing::Test
{
  GalileoSolver *test;
  void SetUp() override {test = new GalileoSolver("");}
  void TearDown() override {delete test;}
};

TEST_F(GalileoSolverTest, CheckSyncHeaders)
{
  uint8_t test_byte1 = 0xb5;
  test->checkSyncHeaders(test_byte1);

  EXPECT_TRUE(test->sync_lock_1_);
}

TEST_F(GalileoSolverTest, CheckSyncHeaders2)
{
  uint8_t test_byte1 = 0xb5;
  uint8_t test_byte2 = 0x62;

  test->checkSyncHeaders(test_byte1);
  test->checkSyncHeaders(test_byte2);

  EXPECT_TRUE(test->sync_lock_1_);
  EXPECT_TRUE(test->sync_lock_2_);
}

TEST_F(GalileoSolverTest, DetermineWordTypeTrue)
{
  GalileoSolver::MessageDataWordHead payload_data_word_head_test;
  std::vector<unsigned int> word_types = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 16, 17, 63};

  for (int word_type : word_types)
  {
    payload_data_word_head_test.word_type = word_type;
    test->determineWordType(payload_data_word_head_test);
    EXPECT_EQ(test->word_type_, word_type);
  }
}

TEST_F(GalileoSolverTest, DetermineWordTypeFalse)
{
  GalileoSolver::MessageDataWordHead payload_data_word_head_test;
  std::vector<unsigned int> word_types = {23, 24, 35, 41};

  for (int word_type : word_types)
  {
    payload_data_word_head_test.word_type = word_type;
    test->determineWordType(payload_data_word_head_test);
    EXPECT_NE(test->word_type_, word_type);
  }
}

/*TEST_F(GalileoSolverTest, GetBits)
{
  std::vector<uint32_t> datawords = {0xc601029a, 0x31f71e26, 0xc0d5d048};
  std::vector<std::vector<uint32_t>> parsed_datawords = {{0xc, 0x6, 0x1, 0x29a},
                                                         {0x3, 0x1, 0xf7, 0x1e26},
                                                         {0xc, 0x0, 0xd5, 0xd048}}


  for (uint32_t dataword : datawords)
  {
    EXPECT_EQ(test->getBits())
  }

}*/


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}