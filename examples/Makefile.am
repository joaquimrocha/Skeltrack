AM_CFLAGS = \
	$(GLIB_CFLAGS) \
	$(EXAMPLES_DEPS_CFLAGS) \
	-I$(top_srcdir)/@PRJ_NAME@

LDADD = \
	$(top_builddir)/@PRJ_NAME@/lib@PRJ_API_NAME@.la \
	$(EXAMPLES_DEPS_LIBS)

noinst_PROGRAMS = \
	test-kinect

test_kinect_SOURCES = test-kinect.c

MAINTAINERCLEANFILES = Makefile.in *~

DISTCLEANFILES = $(MAINTAINERCLEANFILES)