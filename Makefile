# Makefile

# Variables
CXX=clang++
CXXFLAGS=-std=c++20 -Wall -Wextra -O2 -I src
TEST_SRC=test/main.cpp
TEST_BIN=test/main

# Targets
.PHONY: init gen test clean

# Build the test program
$(TEST_BIN): $(TEST_SRC)
	@echo "Building test program"
	$(CXX) $(CXXFLAGS) -o $(TEST_BIN) $(TEST_SRC)

# Run the test program
test: $(TEST_BIN)
	@echo "Running test program"
	$(TEST_BIN)

# Clean up build artifacts
clean:
	@echo "Cleaning up"
	rm -f $(TEST_BIN)
