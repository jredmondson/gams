#ifndef INCLUDED_CPP11_COMPAT_H
#define INCLUDED_CPP11_COMPAT_H

#if __cplusplus >= 201103L
// If C++11 support is active
#else
// If C++11 support is not active

// constexpr simply ignored if not supported
#define constexpr

#define nullptr NULL

#endif

#endif
