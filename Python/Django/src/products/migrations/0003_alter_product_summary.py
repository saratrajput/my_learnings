# Generated by Django 4.1 on 2022-09-06 03:43

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ("products", "0002_alter_product_description_alter_product_price"),
    ]

    operations = [
        migrations.AlterField(
            model_name="product",
            name="summary",
            field=models.TextField(),
        ),
    ]
