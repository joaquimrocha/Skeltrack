SUBDIRS = \
	skeltrack

if ENABLE_TESTS
SUBDIRS += tests
endif

if BUILD_GTK_DOC
SUBDIRS += doc
endif

if ENABLE_EXAMPLES
SUBDIRS += examples
endif

DIST_SUBDIRS = \
	skeltrack

EXTRA_DIST = \
	autogen.sh \
	m4/introspection.m4

ACLOCAL_AMFLAGS = -I m4
DISTCHECK_CONFIGURE_FLAGS = --enable-introspection

CLEANFILES = *~

MAINTAINERCLEANFILES = \
	Makefile.in aclocal.m4 configure config.guess config.sub \
	depcomp install-sh ltmain.sh missing mkinstalldirs config.h.in \
	stamp-h.in compile

DISTCLEANFILES = \
	cscope.files cscope.out

cscope.files:
	find src -name '*.[ch]' > $@

cscope.out: cscope.files
	cscope -b

dist-hook:
	@if test -d "$(srcdir)/.git"; \
	then \
		echo Creating ChangeLog && \
		( cd "$(top_srcdir)" && \
		 $(top_srcdir)/missing --run git log --stat ) > ChangeLog.tmp \
		&& mv -f ChangeLog.tmp $(top_distdir)/ChangeLog \
		|| ( rm -f ChangeLog.tmp ; \
		    echo Failed to generate ChangeLog >&2 ); \
	else \
		echo A git clone is required to generate a ChangeLog >&2; \
	fi
