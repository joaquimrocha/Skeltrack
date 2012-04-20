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

noinst_PROGRAMS = \
	test-skeleton

TESTS = \
	test-skeleton

test_skeleton_SOURCES = test-skeleton.c
