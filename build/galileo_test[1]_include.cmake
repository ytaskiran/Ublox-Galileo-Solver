if(EXISTS "/home/cezeri/galileo/build/galileo_test[1]_tests.cmake")
  include("/home/cezeri/galileo/build/galileo_test[1]_tests.cmake")
else()
  add_test(galileo_test_NOT_BUILT galileo_test_NOT_BUILT)
endif()
