#!/bin/bash
# Create a zip file containing the source code including submodules
# Usage: release <version>

option="$1"
file_name="li-charger"

version_major=$(cat "$file_name.ino" | grep '#define VERSION_MAJOR' | awk -F' ' '{print $3}')
version_minor=$(cat "$file_name.ino" | grep '#define VERSION_MINOR' | awk -F' ' '{print $3}')
version_maint=$(cat "$file_name.ino" | grep '#define VERSION_MAINT' | awk -F' ' '{print $3}')

version="$version_major.$version_minor.$version_maint"

if [[ -z $version ]]; then
  echo "Error: No version string specified"
  exit 1
fi

sed -i -e "s/.* * Version:.*/ * Version: $version/" "$file_name.ino"
sed -i -e "s/.* * Date:.*/ * Date:    $(date '+%B %d, %Y')/" "$file_name.ino"
rm -rf "$file_name.ino-e"
rm -rf "$file_name-"*"-full"*
rm -rf "$file_name"
rm -rf "$file_name "*

make clean

if [[ "$option" != "clean" ]]; then
  cd .. && zip -r "$file_name/$file_name-$version-full.zip" "$file_name" -x '*.git*' '*.vscode*' '*private*' '*build-*' '*.DS_Store*'
fi

exit 0
