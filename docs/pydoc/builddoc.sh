#!/usr/bin/env bash
rm -r build/
sphinx-apidoc -f -o source/ ../../source/rafcontpp
sphinx-build-2.7 -b html source/ build/