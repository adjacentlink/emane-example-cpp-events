ACLOCAL_AMFLAGS = -I m4

SUBDIRS=src

EXTRA_DIST =         \
 AUTHORS             \
 COPYING             \
 ChangeLog           \
 NEWS                \
 README.md           

dist-hook:
	find $(distdir) -name "*.pb.*" -delete

clean-local:
	@rm -f emane-example-cpp-events-*.tar.gz
