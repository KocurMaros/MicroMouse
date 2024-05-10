# EXPORTING python path
after . ./export.sh export this: 

export PYTHONPATH="$PYTHONPATH:$IDF_PATH/tools:$IDF_PATH/tools/ci/python_packages"
# req i needed 
pip install python-gitlab

pip install pexpect

pip install junit_xml

# how to run server

in idf.py menuconfig 

set Example Connection Configuration SSID and Password

and check Example Configuration if port used by ESP is free on your pc arp -a linux

# RUn script

python3 udp_server.py IPv4