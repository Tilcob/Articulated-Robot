#pragma once

#include <stdint.h>
#include "Types.h"

// Stateful serial parser. Call poll() in loop; returns true if a new command was parsed.
class SerialControl {
public:
	// Returns true exactly once per parsed command.
	// - debugPrint toggled by commands
	// - tcpRelative "t x y z" uses last pose if available.
	bool poll(bool& debugPrint,
			  bool havePose,
			  const Vec3& lastTcpPosBase,
			  const Mat3& lastRBaseTcp,
			  Vec3& outPos,
			  float& outGrip01);

	static void printHelp();

private:
	static void skipSpaces_(char*& p);

	static bool parseLine_(char* line,
	                       bool& debugPrint,
	                       bool havePose,
	                       const Vec3& lastTcpPosBase,
	                       const Mat3& lastRBaseTcp,
	                       Vec3& outPos,
	                       float& outGrip01);

	char buf_[96]{};
	uint8_t len_{0};
};
