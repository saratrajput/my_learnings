from django.http import HttpResponse
from django.shortcuts import render

# Create your views here.
def home_view(request, *args, **kwargs):
    print(args, kwargs)
    print(request.user)
    # return HttpResponse("<h1>Hello World</h1>")  # String of HTML code.
    return render(request, "home.html", {})


def contact_view(request, *args, **kwargs):
    return HttpResponse("<h1>Contact Page</h1>")  # String of HTML code.


def about_view(request, *args, **kwargs):
    return HttpResponse("<h1>About Page</h1>")  # String of HTML code.


def social_view(request, *args, **kwargs):
    return HttpResponse("<h1>Social Page</h1>")  # String of HTML code.
