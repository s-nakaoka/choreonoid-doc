#!/bin/bash
rsync -rv ../website/_build/html/ja/manuals/latest/ cnoidsrv:/var/www/choreonoid.org/public/ja/manuals/latest/
