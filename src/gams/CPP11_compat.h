#ifndef INCLUDED_CPP11_COMPAT_H
#define INCLUDED_CPP11_COMPAT_H

#ifdef __GNUC__
#define WARN_UNUSED __attribute__((warn_unused_result))
#else
#define WARN_UNUSED
#endif

#if __cplusplus >= 201103L || _MSC_VER >= 1900
// If C++11 support is active
#define CPP11
#else
// If C++11 support is not active

#if __cpp_constexpr >= 200704
#else
// constexpr simply ignored if not supported
#define constexpr
#endif

// override simply ignored if not supported
#define override

#if __cpp_static_assert >= 200410L
#else
// approximate static_assert as best as possible
#define static_assert(pred, message) \
     /* error: static_assert failure, see reason below */ typedef int sa_fail__\
[(pred) ? 1 : -1]
#endif

#endif

#endif
