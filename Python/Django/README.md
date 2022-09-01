# Djano for Beginners
Free and open-source framework for building web apps with Python.

## Features
* The admin site: For managing data.
* Object-relational mapper(ORM)
    * For querying and persisting data without writing SQL.
* Authentication.
* Caching.

## Setup
```
mkdir src
cd src/

# Create a new project.
django-admin startproject trydjango .

# Launch server
python manage.py runserver

# Migrate (In a separate terminal.)
python manage.py migrate

# Create superuser.
python manage.py createsuperuser
```

### Your First App Component

* Create a new app.
```
python manage.py startapp products
```

* Make your changes in ```models.py```.

* Add your app in ```settings.py```.

```
INSTALLED_APPS = [
    ...,
    "products",
]
```

* Migrate.

```
python manage.py makemigrations
python manage.py migrate
```
> Run the above commands everytime you make changes to ```model.py```.

* Register your model in ```admin.py```.
```
from .models import Product
admin.site.register(Product)
```

#### Create Product Objects in the Python Shell

```
python manage.py shell
```
