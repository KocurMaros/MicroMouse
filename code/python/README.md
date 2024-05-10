# EXPORTING python path
after . ./export.sh export this: 

export PYTHONPATH="$PYTHONPATH:$IDF_PATH/tools:$IDF_PATH/tools/ci/python_packages"
# req i needed 
pip install python-gitlab

pip install pexpect

pip install junit_xml