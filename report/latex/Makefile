# Build the report: pdflatex twice for the table of contents / cross-refs.
#
# The system TeX Live (Ubuntu base) is missing xcolor, tcolorbox, listings,
# titlesec, enumitem, booktabs, caption, pgf. A local copy of those trees lives
# at ~/.texlive-extra/texmf-dist and is added to TEXINPUTS below. If you install
# the real packages (sudo apt install texlive-latex-extra texlive-latex-recommended
# texlive-pictures texlive-fonts-recommended) this still works.

TEXMFEXTRA := $(HOME)/.texlive-extra/texmf-dist
export TEXINPUTS := .:$(TEXMFEXTRA)//:

main.pdf: main.tex preamble.tex $(wildcard sections/*.tex)
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	pdflatex -interaction=nonstopmode -halt-on-error main.tex

.PHONY: clean
clean:
	rm -f main.aux main.log main.out main.toc main.pdf sections/*.aux
	rm -f *.tfm *.gf *.600gf *.600pk ec*.log
