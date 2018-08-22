#!/bin/bash
rsync --rsh="ssh -p 39622" -v -r ../website/_build/html/ja/manuals/latest/ choreonoid.org:/var/www/public/ja/manuals/latest/
