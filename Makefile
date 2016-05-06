OS := $(shell uname)
ARCH := $(shell uname -m)

ifeq ($(OS), Linux)
  ifeq ($(ARCH), x86_64)
    LEAP_LIBRARY := ./lib/x64/libLeap.so -Wl,-rpath,./lib/x64
  else
    LEAP_LIBRARY := ./lib/x86/libLeap.so -Wl,-rpath,./lib/x86
  endif
else
  # OS X
  LEAP_LIBRARY := ./lib/libLeap.dylib
endif

leap_to_hand: leap_to_hand.cpp
	$(CXX) -Wall -g -I./include -std=c++11 leap_to_hand.cpp -o leap_to_hand $(LEAP_LIBRARY)
ifeq ($(OS), Darwin)
	install_name_tool -change @loader_path/libLeap.dylib ./lib/libLeap.dylib leap_to_hand
endif

clean:
	rm -rf leap_to_hand leap_to_hand.dSYM
