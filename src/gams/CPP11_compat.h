#ifndef INCLUDED_CPP11_COMPAT_H
#define INCLUDED_CPP11_COMPAT_H

#if __cplusplus >= 201103L || (MSC_VER >= 1600) 
// If C++11 support is active
#define CPP11
#else
// If C++11 support is not active

// constexpr simply ignored if not supported
#define constexpr

// override simply ignored if not supported
#define override

// approximate static_assert as best as possible
#define static_assert(pred, message) \
     /* error: static_assert failure, see reason below */ typedef int sa_fail__\
[(pred) ? 1 : -1]

#define nullptr NULL

#endif

#endif
