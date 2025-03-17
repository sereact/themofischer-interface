#!/bin/bash
./build_frontend.sh
rm -rf wheelhouse
python setup.py bdist_wheel -d wheelhouse
docker build -t thermofischer-interface .