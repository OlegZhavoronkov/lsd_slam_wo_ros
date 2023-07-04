LSD-SLAM Documentation is written in
[Sphinx](http://www.sphinx-doc.org/en/master/) and hosted at
[Read The Docs](https://lsd-slam.readthedocs.io/en/latest/).

Everything is coordinated through the Makefile in this directory.  In addition
to the standard Sphinx targets, we've added:

  - `make uml` uses a Docker-ized version of PlantUML to translate the UML
  descriptions in `uml/` to diagrams in `_static/uml/`
  - `make all` combines `make uml` and `make html`
