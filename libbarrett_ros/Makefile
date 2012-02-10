all: installed

SVN_DIR = libbarrett
SVN_URL = http://web.barrett.com/svn/libbarrett/trunk/
#SVN_REVISION = -r 808
#SVN_PATCH = 
include $(shell rospack find mk)/svn_checkout.mk
SOURCE_DIR = $(SVN_DIR) 


installed: wiped $(SOURCE_DIR)
	cd $(SOURCE_DIR) && cmake .
	cd $(SOURCE_DIR) && make
	cd $(SOURCE_DIR) && make install
	touch installed

clean: 
	rm -rf $(SVN_DIR) installed patched wiped rospack_nosubdirs 


wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build cleaned
