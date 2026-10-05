DOXYGEN ?= doxygen
DOXYFILE ?= Doxyfile
OUT_DIR ?= _build

.PHONY: all html latex clean help

all: html

html:
	$(DOXYGEN) $(DOXYFILE)
	@echo "Documentation generated in $(OUT_DIR)/html"

latex:
	$(DOXYGEN) $(DOXYFILE)
	@echo "LaTeX documentation generated in $(OUT_DIR)/latex"

clean:
	rm -rf $(OUT_DIR)

help:
	@echo "Available targets: all, html, latex, clean"
