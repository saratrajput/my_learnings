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
# Go to root directory and launch shell
cd src/
python manage.py shell

# Absolute imports
from products.models import Product

# List all objects
Product.objects.all()

# Create new object
Product.objects.create(title='New Product 2', description='another one', price='19312', summary='sweet')
```

### New Model Fields
> To start over, delete all the files in the ```migrations``` folder and the ```db.sqlite3 file.```.

* [Field-types docs](https://docs.djangoproject.com/en/4.1/ref/models/fields/#field-types).

* ```CharField()```: max_length is **required**.

### Change a Model
* ```blank```: How the field is rendered.
* ```null```: Related to the database.

### Default Homepage to Custom Homegage
* Create a new app called **pages**.
```
python manage.py startapp pages
```

* Add it to ```settings.py```.
```
INSTALLED_APPS = [
    ...,
    "pages",
]
```

* Create a new function with the HTTP text in ```pages/views.py```.

* Import the view in the ```url.py```.

### URL Routing and Requests

* In the ```views.py```, each of the functions ```home_view```, ```contact_view```, etc. has a ```request``` argument by default.

### Django Templates

* We can use render from ```django.shortcuts``` to return a ```.html``` file instead of a just returning some strings.

```
render(request, <template_name>, {context})
```

* In our root directory, create a directory named ```templates```.

* Store ```home.html``` in this directory.

* Add path to your ```templates``` directory in ```settings.py```.

```
TEMPLATES = [
    {
        ...,
        "DIRS": [os.path.join(BASE_DIR, "templates")],
        ...,
    }
]
```

### Django Templating Engine Basics

Inheritance allows us to remove redundant code.

* Create a ```base.html``` file.

* Add content blocks that you would later replace in other files which inherit from ```base.html``` file.

```
{% block content %}
replace me
{% endblock %}
```

* Inherit from ```base.html``` in other html files with: ```{% extends 'base.html' %}```

### Include Template Tag

We can do the same inheritance as above, but using the ```include``` tag.

* Create a new html file called ```navbar.html```.

* Include this in the ```base```.html file with the ```include``` tag.

```
{% include 'navbar.html' %}
```

### Rendering Context in a Template

A context is a variable name -> variable value mapping that is passed to a template. Context processors let you specify a number of variables that get set in each context automatically - without you having to specify the variables in each render() call.

In the ```render()``` function call, you can pass a dictionary whose keys can be called in the corresponding html page to display the values of that dictionary. For example, in the function for ```about.html``` rendering in ```views.py```.

```
def about_view(request, *args, **kwargs):
    # return HttpResponse("<h1>About Page</h1>")  # String of HTML code.
    my_context = {
        "my_text": "This is about us.",
        "my_number": 123,
        "my_list": [133, 21343, 2343],
    }
    return render(request, "about.html", my_context)
```

And, to display these values, in about.html:

```
<p>
    {{ my_text }}, {{ my_number }}
    {{ my_list }}
</p>
```

### For Loop in a Template

In the "html" file.

```
    {% for my_sub_item in my_list %}
    <li>{{ my_sub_item}}</li>
    {% endfor %}  # You need to close the for loop once it's opened.
```

```{{forloop.counter}}``` can be used to see the iteration number.

### Using Conditions in a Template

Example:
```
{% if abc == 312 %}
    <li>{{ forloop.counter }} - {{ abc|add:22}}</li>
{% elif abc == "Abc" %}
    <li>This is not the network.</li>
{% else %}
    <li>{{ forloop.counter }} - {{ abc }}</li>
{% endif %}
```