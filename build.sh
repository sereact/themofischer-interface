#!/bin/bash
rm -rf wheelhouse
python setup.py bdist_wheel -d wheelhouse
docker build -t thermofischer-interface .