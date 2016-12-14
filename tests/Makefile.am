MAINTAINERCLEANFILES = \
	Makefile.in

tests_dir = `pwd`/

AM_CFLAGS = \
	$(GLIB_CFLAGS) \
	-I$(top_srcdir)/@PRJ_NAME@

if ENABLE_DEBUG
AM_CFLAGS += -Werror -g3 -O0 -ggdb
endif

LDADD = \
	$(top_builddir)/@PRJ_NAME@/lib@PRJ_API_NAME@.la
AM_LDFLAGS = -lgio-2.0 -lgobject-2.0 -lglib-2.0
noinst_PROGRAMS = \
	test-skeleton

TESTS = \
	test-skeleton

test_skeleton_SOURCES = test-skeleton.c
