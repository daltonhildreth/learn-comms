#pragma once
#include <cassert>
#ifdef DEBUG

// GLCHECK macro setup
#    define STRING(X) #    X
#    define TOSTRING(X) STRING(X)
void glcheck(const char* loc);
#    define GLCHECK glcheck("@" __FILE__ ":" TOSTRING(__LINE__));

// debug only expression
#    define D(x) x

#else
// do nothing when GLCHECK is called in NDEBUG
#    define GLCHECK

// debug only expressions dissapear!
#    define D(x)

#endif // DEBUG
// variables I want to have in parameters, but are otherwise unused.
template <class... Args> inline void UNUSED(Args&&...) {}
