DESTDIR = 
prefix = /usr/local
exec_prefix = /usr/local
bindir = $(exec_prefix)/bin
sysconfdir = $(prefix)/etc
envdir = $(sysconfdir)/default


src/soyocontroller.service: src/soyocontroller.service.in
	sed \
		-e 's,@bindir@,$(bindir),g' \
		-e 's,@sysconfdir@,$(sysconfdir),g' \
		src/soyocontroller.service.in \
	> src/soyocontroller.service

all: src/soyocontroller.service

clean:
	$(RM) src/soyocontroller.service

install: all
	install -D -m 0755 src/soyocontroller.py $(DESTDIR)$(bindir)/soyocontroller
	install -D -m 0644 src/soyocontroller.service $(DESTDIR)/lib/systemd/system
	install -D -m 0644 src/soyocontroller.env $(DESTDIR)$(sysconfdir)/default/soyocontroller

uninstall:
	$(RM) $(DESTDIR)$(bindir)/soyocontroller
	$(RM) $(DESTDIR)/lib/systemd/system/soyocontroller
	$(RM) $(DESTDIR)$(envdir)/soyocontroller

.PHONY: all clean install uninstall
