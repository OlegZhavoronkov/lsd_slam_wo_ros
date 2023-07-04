
# To make base64 encoded images

convert 00001.png -depth 8  gray:- | base64 > 00001.base64


or use the `make_testdata.py` script which will make the source file `testimages_data.cpp`
