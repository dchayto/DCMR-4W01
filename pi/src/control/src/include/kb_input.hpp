#ifndef PI_CONTROL_INPUTS
#define PI_CONTROL_INPUTS

#include <cstdint>

namespace InputConstants	{
	inline constexpr double ISR2	{ 0.707 }; // 1/sqrt(2)
}

namespace KeyboardConstants	{
	// basic movements
	inline constexpr char FWD 		{'w'};
	inline constexpr char LFT	 	{'a'};
	inline constexpr char RHT 		{'d'};
	inline constexpr char REV 		{'s'};
	inline constexpr char FWD_RHT	{'e'};
	inline constexpr char FWD_LFT 	{'q'};
	inline constexpr char REV_RHT 	{'c'};
	inline constexpr char REV_LFT 	{'z'};
	inline constexpr char STP 		{'x'};

	inline constexpr char TRN_LFT	{','};
	inline constexpr char TRN_RHT	{'.'};
	
	// gain adjustment
	inline constexpr char GAINUP	{']'};
	inline constexpr char GAINUP10	{'}'};
	inline constexpr char GAINDN	{'['};
	inline constexpr char GAINDN10	{'{'};
	
	// quit program
	inline constexpr char QT		{'~'};
}

#endif
