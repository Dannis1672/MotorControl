/* config.h — Platform-adaptive for libmodbus 3.1.6
 * Supports: Windows (MSVC/MinGW) and Linux (GCC/Clang)
 */

#ifndef LIBMODBUS_CONFIG_H
#define LIBMODBUS_CONFIG_H

/* ── Platform detection ─────────────────────────────────── */
#if defined(_WIN32)
  /* Windows */
  #define HAVE_WINSOCK2_H    1
  #define HAVE_ERRNO_H        1
  #define HAVE_FCNTL_H        1
  #define HAVE_INTTYPES_H     1
  #define HAVE_LIMITS_H       1
  #define HAVE_MEMORY_H       1
  #define HAVE_MEMSET         1
  #define HAVE_STDINT_H       1
  #define HAVE_STDLIB_H       1
  #define HAVE_STRERROR       1
  #define HAVE_STRING_H       1
  #define HAVE_SYS_STAT_H     1
  #define HAVE_SYS_TYPES_H    1
  #define HAVE_TIME_H         1

  #undef  HAVE_ARPA_INET_H
  #undef  HAVE_DECL_TIOCSRS485
  #undef  HAVE_DECL___CYGWIN__
  #undef  HAVE_DLFCN_H
  #undef  HAVE_FORK
  #undef  HAVE_GETADDRINFO
  #undef  HAVE_GETTIMEOFDAY
  #undef  HAVE_INET_NTOA
  #undef  HAVE_LINUX_SERIAL_H
  #undef  HAVE_NETDB_H
  #undef  HAVE_NETINET_IN_H
  #undef  HAVE_NETINET_TCP_H
  #undef  HAVE_SELECT
  #undef  HAVE_SOCKET
  #undef  HAVE_STRINGS_H
  #undef  HAVE_STRLCPY
  #undef  HAVE_SYS_IOCTL_H
  #undef  HAVE_SYS_SOCKET_H
  #undef  HAVE_SYS_TIME_H
  #undef  HAVE_TERMIOS_H
  #undef  HAVE_UNISTD_H
  #undef  HAVE_VFORK
  #undef  HAVE_VFORK_H
  #undef  HAVE_WORKING_FORK
  #undef  TIME_WITH_SYS_TIME

#else
  /* Linux / Unix */
  #define HAVE_ARPA_INET_H     1
  #define HAVE_DECL_TIOCSRS485 1
  #define HAVE_DLFCN_H         1
  #define HAVE_ERRNO_H         1
  #define HAVE_FCNTL_H         1
  #define HAVE_FORK            1
  #define HAVE_GETADDRINFO     1
  #define HAVE_GETTIMEOFDAY    1
  #define HAVE_INET_NTOA       1
  #define HAVE_INTTYPES_H      1
  #define HAVE_LIMITS_H        1
  #define HAVE_LINUX_SERIAL_H  1
  #define HAVE_MEMORY_H        1
  #define HAVE_MEMSET          1
  #define HAVE_NETDB_H         1
  #define HAVE_NETINET_IN_H    1
  #define HAVE_NETINET_TCP_H   1
  #define HAVE_SELECT          1
  #define HAVE_SOCKET          1
  #define HAVE_STDINT_H        1
  #define HAVE_STDLIB_H        1
  #define HAVE_STRERROR        1
  #define HAVE_STRINGS_H       1
  #define HAVE_STRING_H        1
  #define HAVE_SYS_IOCTL_H     1
  #define HAVE_SYS_SOCKET_H    1
  #define HAVE_SYS_STAT_H      1
  #define HAVE_SYS_TIME_H      1
  #define HAVE_SYS_TYPES_H     1
  #define HAVE_TERMIOS_H       1
  #define HAVE_TIME_H          1
  #define HAVE_UNISTD_H        1
  #define HAVE_WORKING_FORK    1

  #undef  HAVE_WINSOCK2_H
  #undef  HAVE_DECL___CYGWIN__
  #undef  HAVE_STRLCPY
  #undef  HAVE_VFORK
  #undef  HAVE_VFORK_H
  #undef  TIME_WITH_SYS_TIME

#endif

/* ── Common (platform-independent) ──────────────────────── */
#define STDC_HEADERS 1

#define PACKAGE                 "libmodbus"
#define PACKAGE_BUGREPORT       "https://github.com/stephane/libmodbus/issues"
#define PACKAGE_NAME            "libmodbus"
#define PACKAGE_STRING          "libmodbus 3.1.6"
#define PACKAGE_TARNAME         "libmodbus"
#define PACKAGE_URL             ""
#define PACKAGE_VERSION         "3.1.6"
#define VERSION                 "3.1.6"

/* sys/types.h fallbacks (both platforms) */
/* #undef const */
/* #undef pid_t */
/* #undef size_t */
#define vfork fork

#endif /* LIBMODBUS_CONFIG_H */
