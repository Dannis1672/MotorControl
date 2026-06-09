/* config.h — 条件编译支持 MSVC (Windows) 与 GCC (Ubuntu 24.04 Linux) */
/* Generated from config.h.in by configure; manually adapted for multi-platform. */

#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

/* ── 平台检测 ──────────────────────────────────────────────────── */
#if defined(_MSC_VER)
#  define MODBUS_PLATFORM_WINDOWS 1
#elif defined(__linux__)
#  define MODBUS_PLATFORM_LINUX 1
#else
#  warning "Unknown platform — edit config.h to add support."
#endif

/* ──────────────────────────────────────────────────────────────────
 * 以下宏定义使用条件编译，MSVC 和 Linux GCC 各自取值。
 * ────────────────────────────────────────────────────────────────── */

/* ---- 两平台通用 ---- */
#define HAVE_ERRNO_H    1
#define HAVE_FCNTL_H    1
#define HAVE_INTTYPES_H 1
#define HAVE_LIMITS_H   1
#define HAVE_MEMORY_H   1
#define HAVE_MEMSET     1
#define HAVE_STDINT_H   1
#define HAVE_STDLIB_H   1
#define HAVE_STRERROR   1
#define HAVE_STRING_H   1
#define HAVE_SYS_STAT_H 1
#define HAVE_SYS_TYPES_H 1
#define HAVE_TIME_H     1
#define STDC_HEADERS    1

/* ---- Linux 专有 ---- */
#ifdef MODBUS_PLATFORM_LINUX
#  define HAVE_ARPA_INET_H        1
#  define HAVE_DECL_TIOCSRS485    1
#  define HAVE_DECL___CYGWIN__    0
#  define HAVE_DLFCN_H            1
#  define HAVE_FORK               1
#  define HAVE_GETADDRINFO        1
#  define HAVE_GETTIMEOFDAY       1
#  define HAVE_LINUX_SERIAL_H     1
#  define HAVE_NETDB_H            1
#  define HAVE_NETINET_IN_H       1
#  define HAVE_NETINET_IP_H       1
#  define HAVE_NETINET_TCP_H      1
#  define HAVE_SELECT             1
#  define HAVE_SOCKET             1
#  define HAVE_STRINGS_H          1
#  define HAVE_SYS_IOCTL_H        1
#  define HAVE_SYS_SOCKET_H       1
#  define HAVE_SYS_TIME_H         1
#  define HAVE_TERMIOS_H          1
#  define HAVE_UNISTD_H           1
#  define HAVE_VFORK              1
#  define HAVE_VFORK_H            1
#  define HAVE_WORKING_FORK       1
#  define HAVE_WORKING_VFORK      1
#  define TIME_WITH_SYS_TIME      1
#  define vfork                  fork
#endif

/* ---- Windows (MSVC) 专有 ---- */
#ifdef MODBUS_PLATFORM_WINDOWS
#  define HAVE_ARPA_INET_H        0
#  define HAVE_DECL_TIOCSRS485    0
#  define HAVE_DECL___CYGWIN__    0
#  define HAVE_DLFCN_H            0
#  define HAVE_FORK               0
#  define HAVE_GETADDRINFO        0
#  define HAVE_GETTIMEOFDAY       0
#  define HAVE_LINUX_SERIAL_H     0
#  define HAVE_NETDB_H            0
#  define HAVE_NETINET_IN_H       0
#  define HAVE_NETINET_TCP_H      0
#  define HAVE_SELECT             0
#  define HAVE_SOCKET             0
#  define HAVE_STRINGS_H          0
/* Windows 有 winsock2.h 而非 sys/socket.h */
#  define HAVE_SYS_IOCTL_H        0
#  define HAVE_SYS_SOCKET_H       0
#  define HAVE_SYS_TIME_H         0
#  define HAVE_TERMIOS_H          0
#  define HAVE_UNISTD_H           0
#  define HAVE_VFORK              0
#  define HAVE_VFORK_H            0
#  define HAVE_WORKING_FORK       0
#  define HAVE_WORKING_VFORK      0
#  define TIME_WITH_SYS_TIME      0
#  define HAVE_WINSOCK2_H         1
#endif

/* ---- 两个平台都未定义的功能 ---- */
/* strlcpy 在 glibc ≥ 2.38 中可用 (Ubuntu 24.04 为 glibc 2.39)，
   但需要 _DEFAULT_SOURCE；为兼容性保留 undef。 */
#ifndef HAVE_STRLCPY
/* #undef HAVE_STRLCPY */
#endif

/* ---- 包信息 (两平台通用) ---- */
#define PACKAGE           "libmodbus"
#define PACKAGE_BUGREPORT "https://github.com/stephane/libmodbus/issues"
#define PACKAGE_NAME      "libmodbus"
#define PACKAGE_STRING    "libmodbus 3.1.12"
#define PACKAGE_TARNAME   "libmodbus"
#define PACKAGE_URL       ""
#define PACKAGE_VERSION   "3.1.12"
#define VERSION           "3.1.12"

/* ---- autotools 遗留宏 ---- */
/* #undef LT_OBJDIR */
/* #undef const */
/* #undef pid_t */
/* #undef size_t */

#endif /* MODBUS_CONFIG_H */
