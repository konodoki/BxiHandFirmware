// HandConfig.h
#pragma once

// Choose one via build flags OR by uncommenting a line below:
//   - PlatformIO:   build_flags = -DRIGHT_HAND   (or -DLEFT_HAND)
//   - Arduino CLI:  --build-property compiler.cpp.extra_flags="-DRIGHT_HAND"
//   - Arduino IDE:  just uncomment one of the lines here.

#define BXI_HAND_V1
// #define BXI_HAND_V2

#if !defined(BXI_HAND_V1) && !defined(BXI_HAND_V2)
  #error "Define exactly one: BXI_HAND_V1 or BXI_HAND_V2 (build flag or uncomment in HandConfig.h)."
#endif

#if defined(BXI_HAND_V1) && defined(BXI_HAND_V2)
  #error "Do not define both BXI_HAND_V1 and BXI_HAND_V2."
#endif

#define LEFT_HAND
// #define RIGHT_HAND

#if !defined(LEFT_HAND) && !defined(RIGHT_HAND)
  #error "Define exactly one: LEFT_HAND or RIGHT_HAND (build flag or uncomment in HandConfig.h)."
#endif

#if defined(LEFT_HAND) && defined(RIGHT_HAND)
  #error "Do not define both LEFT_HAND and RIGHT_HAND."
#endif