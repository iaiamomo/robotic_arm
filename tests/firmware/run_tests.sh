#!/bin/sh
# Compile the firmware against host-side mocks and run the parser tests.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -I. -Wall -Wextra -Wno-unused-parameter -x c++ -include Braccio++.h test_parser.cpp -o /tmp/fw_parser_test
/tmp/fw_parser_test
