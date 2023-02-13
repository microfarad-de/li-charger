#!/bin/bash
# Create a zip file containing the source code including submodules
# Usage: release <version>

option="$1"
dir_name=$(basename $(pwd))

if [[ "$option" == "clean" ]]; then
  rm -rf "$dir_name-"*"-full"*
  rm -rf "$dir_name"
  rm -rf "$dir_name "*
else
  version_major=$(cat "$dir_name.ino" | grep '#define VERSION_MAJOR' | awk -F' ' '{print $3}')
  version_minor=$(cat "$dir_name.ino" | grep '#define VERSION_MINOR' | awk -F' ' '{print $3}')
  version_maint=$(cat "$dir_name.ino" | grep '#define VERSION_MAINT' | awk -F' ' '{print $3}')
  version="$version_major.$version_minor.$version_maint"
  if [[ -z $version ]]; then
    echo "Error: No version string specified"
    exit 1
  fi
  sed -i -e "s/.* * Version:.*/ * Version: $version/" "$dir_name.ino"
  sed -i -e "s/.* * Date:.*/ * Date:    $(date '+%B %d, %Y')/" "$dir_name.ino"
  rm -rf "$dir_name.ino-e"
  cd .. && zip -r "$dir_name/$dir_name-$version-full.zip" "$dir_name" -x '*.git*' '*.vscode*' '*private*' '*build-*' '*.DS_Store*'
fi

exit 0
