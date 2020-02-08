rm -rf ./venv
python3 -m venv ./venv
source ./venv/bin/activate
./install_dependencies.sh
echo "Run 'source ./venv/bin/activate' to activate the virtual environment."
