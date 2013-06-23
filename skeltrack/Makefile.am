MAINTAINERCLEANFILES = \
	Makefile.in

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = @PRJ_API_NAME@.pc

library_includedir=$(includedir)/@PRJ_API_NAME@
library_include_HEADERS = skeltrack.h

CLEANFILES = *~ @PRJ_API_NAME@.pc

AM_CFLAGS = $(GLIB_CFLAGS) -Wall \
	-DPKGDATADIR="\"$(pkgdatadir)\"" \
	-DENABLE_TESTS="\"$(enable_tests)\""

if ENABLE_DEBUG
AM_CFLAGS += -Werror -g3 -O0 -ggdb
else
AM_CFLAGS += -DG_DISABLE_ASSERT -DG_DISABLE_CHECKS
endif

# libraries
lib_LTLIBRARIES = lib@PRJ_API_NAME@.la

# libskeltrack
source_c = \
	skeltrack-joint.c \
	skeltrack-skeleton.c \
	skeltrack-smooth.c \
	skeltrack-util.c \
	pqueue.c

source_h = \
	skeltrack.h \
	skeltrack-joint.h \
	skeltrack-skeleton.h

source_h_priv =

lib@PRJ_API_NAME@_la_LIBADD = \
	$(GLIB_LIBS) \
	-lm

lib@PRJ_API_NAME@_la_CFLAGS  = \
	$(AM_CFLAGS)

lib@PRJ_API_NAME@_la_LDFLAGS = \
	-version-info 0:1:0 \
	-no-undefined

lib@PRJ_API_NAME@_la_SOURCES = \
	$(source_c) \
	$(source_h) \
	$(source_h_priv)

skeltrackdir = $(includedir)/@PRJ_API_NAME@
skeltrack_HEADERS = \
	$(source_h) \
	$(source_h_priv)

noinst_HEADERS = skeltrack-smooth.h skeltrack-util.h pqueue.h

# introspection support
if HAVE_INTROSPECTION

-include $(INTROSPECTION_MAKEFILE)
INTROSPECTION_GIRS =
INTROSPECTION_SCANNER_ARGS = --add-include-path=$(srcdir) --warn-all --symbol-prefix=skeltrack
INTROSPECTION_COMPILER_ARGS = --includedir=$(srcdir)

introspection_sources = $(skeltrack_HEADERS) $(source_c)

@PRJ_GIR_API_NAME@.gir: lib@PRJ_API_NAME@.la Makefile
@PRJ_GIR_TARGET_NAME@_INCLUDES = GObject-2.0 Gio-2.0
@PRJ_GIR_TARGET_NAME@_CFLAGS = $(INCLUDES)
@PRJ_GIR_TARGET_NAME@_LIBS = @PRJ_API_NAME@ m
@PRJ_GIR_TARGET_NAME@_NAMESPACE = @PRJ_GIR_NAMESPACE@
@PRJ_GIR_TARGET_NAME@_FILES = $(addprefix $(srcdir)/,$(introspection_sources))
@PRJ_GIR_TARGET_NAME@_SCANNERFLAGS = \
	--warn-all \
	--c-include='skeltrack.h' \
	--pkg-export=@PRJ_API_NAME@

INTROSPECTION_GIRS += @PRJ_GIR_API_NAME@.gir

girdir = @INTROSPECTION_GIRDIR@
gir_DATA = $(INTROSPECTION_GIRS)

typelibdir = @INTROSPECTION_TYPELIBDIR@
typelib_DATA = $(INTROSPECTION_GIRS:.gir=.typelib)

CLEANFILES += $(dist_gir_DATA) $(typelib_DATA)
endif

maintainer-clean-local:
	rm -rf tmp-introspect*
