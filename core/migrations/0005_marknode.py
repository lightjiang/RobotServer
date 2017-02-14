# -*- coding: utf-8 -*-
# Generated by Django 1.9.4 on 2017-02-13 13:05
from __future__ import unicode_literals

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ('core', '0004_robot'),
    ]

    operations = [
        migrations.CreateModel(
            name='MarkNode',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('change_time', models.DateTimeField(auto_now=True)),
                ('change_code', models.CharField(default='', max_length=32)),
                ('create_time', models.DateTimeField(auto_now_add=True)),
                ('label', models.CharField(default='', max_length=50)),
                ('status', models.BooleanField(default=True)),
                ('x', models.FloatField(default=0.0)),
                ('y', models.FloatField(default=0.0)),
                ('z', models.FloatField(default=0.0)),
                ('is_shown', models.BooleanField(default=True)),
                ('robot', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, to='core.Robot')),
            ],
            options={
                'abstract': False,
            },
        ),
    ]