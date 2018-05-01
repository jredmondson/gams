#ifndef INCLUDED_CPP11_COMPAT_H
#define INCLUDED_CPP11_COMPAT_H

#ifdef __GNUC__
#define WARN_UNUSED __attribute__((warn_unused_result))
#else
#define WARN_UNUSED
#endif

#endif
