# -*- coding: utf-8 -*-
# Generated by Django 1.9.4 on 2017-02-13 12:45
from __future__ import unicode_literals

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('core', '0002_accesstrack_robot'),
    ]

    operations = [
        migrations.DeleteModel(
            name='Robot',
        ),
    ]