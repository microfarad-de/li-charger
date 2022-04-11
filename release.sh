#!/bin/bash
# Create a zip file containing the source code including submodules
# Usage: release <version>

version="$1"
file_name="li-charger"

if [[ -z $version ]]; then
  echo "Error: No version string specified"
  echo "Usage: $0 <version>"
  exit 1
fi

rm -r "$file_name-"*"-full"*

if [[ $version != "clean" ]]; then
  zip -r "$file_name-$version-full.zip" . -x '*.git*' '*.vscode*' '*private*' '*.DS_Store'
fi

exit 0
