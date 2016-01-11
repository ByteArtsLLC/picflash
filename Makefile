# Makefile for building picflash on Raspberry Pi

CC       = gcc
CFLAGS   = -O0 -g -Wall -marm
LDFLAGS  = -lusb-1.0 -marm
EXEC     = picflash
EXECPATH = pi_linux
SYSTEM = linux

OBJS = Comm.o \
       Delays.o \
       Device.o \
       DeviceData.o \
       HexFileIO.o \
       hid-libusb-linux.c \
       ImportExportHex.o \
       Logger.o \
       main.o

HDRS = Comm.h \
       Delays.h \
       Device.h \
       DeviceData.h \
       HexFileIO.h \
       hidapi.h \
       ImportExportHex.h \
       Logger.h


all: 
	@echo
	@echo Please make 'picflash' 
	@echo

*.o: $(HDRS)

.c.o:
	$(CC) $(CFLAGS) -c $*.c

#picflash: CFLAGS += -marm 
#picflash: LDFLAGS += -marm
#picflash: EXEC = picflash
#picflash: picflash

picflash: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(EXECPATH)/$(EXEC)
	strip $(EXECPATH)/$(EXEC)


# Must install as root; e.g. 'sudo make install'
install: picflash
	cp $(EXECPATH)/$(EXEC) /usr/local/bin/picflash

	
clean:
	rm -f *.o core


#bindist: tarball zipfile

#srcdist: src-tarball src-zipfile

#tarball:
#	tar cvzf $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-bin.tar.gz README.txt CHANGELOG COPYING $(EXECPATH)/*$(VERSION_MAIN).$(VERSION_SUB)*

#src-tarball:
#	tar cvzf $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-src.tar.gz README.txt CHANGELOG COPYING Makefile* *.c *.h

#zipfile:
#	rm -f $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-bin.zip
#	zip $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-bin.zip README.txt CHANGELOG COPYING $(EXECPATH)/*$(VERSION_MAIN).$(VERSION_SUB)*

#src-zipfile:
#	rm -f $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-src.zip
#	zip $(DISTPATH)/picflash-$(VERSION_MAIN).$(VERSION_SUB)-src.zip README.txt CHANGELOG COPYING Makefile* *.c *.h

