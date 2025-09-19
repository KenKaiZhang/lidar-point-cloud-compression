#!/bin/bash

# A simple shell script to read and display the contents of a Python pickle file.
# It uses the Python interpreter to perform the actual unpickling.

# --- Script Usage ---
# To use this script, save it as 'read_pickle.sh' and make it executable:
# chmod +x read_pickle.sh
#
# Then run it with a pickle file as an argument:
# ./read_pickle.sh my_data.pkl
#
# NOTE: This script requires a Python installation on your system.

# Check if a filename was provided as an argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <pickle_file>"
    exit 1
fi

PICKLE_FILE=$1

# Check if the file exists
if [ ! -f "$PICKLE_FILE" ]; then
    echo "Error: File not found -> $PICKLE_FILE"
    exit 1
fi

echo "--- Reading contents of $PICKLE_FILE ---"
echo ""

# Use a Python one-liner to load and pretty-print the contents of the pickle file.
# We import 'pickle' for unpickling, 'pprint' for pretty printing, and 'sys' to access arguments.
# A try/except block is used for robust error handling in case the file is not a valid pickle.
python -c "
import pickle
import sys
import pprint

try:
    with open(sys.argv[1], 'rb') as f:
        data = pickle.load(f)
    print('Unpickled data:')
    pprint.pprint(data)
except FileNotFoundError:
    print(f'Error: The file {sys.argv[1]} was not found.')
except pickle.UnpicklingError:
    print(f'Error: The file {sys.argv[1]} is not a valid pickle file.')
except Exception as e:
    print(f'An unexpected error occurred: {e}')
" "$PICKLE_FILE"
