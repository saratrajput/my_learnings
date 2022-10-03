from django.http import HttpResponse
from django.shortcuts import render

# Create your views here.
def home_view(request, *args, **kwargs):
    print(args, kwargs)
    print(request.user)
    # return HttpResponse("<h1>Hello World</h1>")  # String of HTML code.
    return render(request, "home.html", {})


def contact_view(request, *args, **kwargs):
    # return HttpResponse("<h1>Contact Page</h1>")  # String of HTML code.
    return render(request, "contact.html", {})


def about_view(request, *args, **kwargs):
    # return HttpResponse("<h1>About Page</h1>")  # String of HTML code.
    my_context = {
        "my_text": "This is about us.",
        "my_number": 123,
        "my_list": [133, 21343, 2343],
    }
    return render(request, "about.html", my_context)


def social_view(request, *args, **kwargs):
    # return HttpResponse("<h1>Social Page</h1>")  # String of HTML code.
    return render(request, "social.html", {})
