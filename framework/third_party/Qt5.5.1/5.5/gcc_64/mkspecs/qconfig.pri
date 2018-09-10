#configuration
CONFIG +=  shared qpa no_mocdepend release qt_no_framework
host_build {
    QT_ARCH = x86_64
    QT_TARGET_ARCH = x86_64
} else {
    QT_ARCH = x86_64
    QMAKE_DEFAULT_LIBDIRS  = 
    QMAKE_DEFAULT_INCDIRS  = 
}
QT_CONFIG +=  minimal-config small-config medium-config large-config full-config build_all debug_and_release gtk2 gtkstyle fontconfig evdev xlib xrender xcb-plugin xcb-qt xcb-xlib xcb-sm xkbcommon-qt accessibility-atspi-bridge linuxfb c++11 accessibility opengl shared qpa reduce_exports reduce_relocations clock-gettime clock-monotonic posix_fallocate mremap getaddrinfo ipv6ifname getifaddrs inotify eventfd png system-freetype harfbuzz zlib nis cups iconv glib dbus openssl xcb xinput2 rpath alsa pulseaudio gstreamer-0.10 icu concurrent audio-backend debug release

#versioning
QT_VERSION = 5.5.1
QT_MAJOR_VERSION = 5
QT_MINOR_VERSION = 5
QT_PATCH_VERSION = 1

#namespaces
QT_LIBINFIX = 
QT_NAMESPACE = 

QT_EDITION = OpenSource
QT_LICHECK = licheck64
QT_RELEASE_DATE = 2015-10-12

QMAKE_RPATHDIR +=  "/home/work/baidu/adu-lab/cybertron-apollo/third-party/Qt5.5.1/5.5/gcc_64"
QT_GCC_MAJOR_VERSION = 4
QT_GCC_MINOR_VERSION = 9
QT_GCC_PATCH_VERSION = 1
