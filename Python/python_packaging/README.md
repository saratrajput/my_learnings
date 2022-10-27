# Example Package

This is a simple example package. You can use
[Github-flavored Markdown](https://guides.github.com/features/mastering-markdown/)
to write your content.

## Uploading a package with twine

pip install twine
* Twine check.

```
twine check dist/*
```

* Twine upload dist.
```
twine upload dist/*
Enter username:
Enter password:
```

### Things to checkout

### Extra credit
* Badges
    * Code coverage (Coveralls, codecov.io).
    * Quality metrics (Code climate, landscape.io)
* Manage versioning with bumpversion.
* Test on OSX & Windows.
* More documentation.
    * Contributors Section.


### With cookiecutter

```
pip install cookiecutter
cookiecutter gh:ionelmc/cookiecutter-pylibrary
```
- Answer the questions.
- Copy in your code and tests.
- Some minor file tweaks.
- DONE!

### Things are changing

* Move metadat from setup.py to setup.cfg.

* Move to pyproject.toml.
    * Poetry (cookiecutter & virtualenv & setup.py).
    * Flit (setup.py).
    * Hatch (cookiecutter & virtualenv & twine).
