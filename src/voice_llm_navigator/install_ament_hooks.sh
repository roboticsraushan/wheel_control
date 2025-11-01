#!/bin/bash
# Post-install hook to ensure ament_prefix_path hooks exist
# This script is called after colcon build to fix missing ament hooks

INSTALL_DIR="$1"
PACKAGE_NAME="voice_llm_navigator"

if [ -z "$INSTALL_DIR" ]; then
    echo "Usage: $0 <install_dir>"
    exit 1
fi

HOOK_DIR="$INSTALL_DIR/share/$PACKAGE_NAME/hook"
PACKAGE_DSV="$INSTALL_DIR/share/$PACKAGE_NAME/package.dsv"

# Create hook directory if it doesn't exist
mkdir -p "$HOOK_DIR"

# Create ament_prefix_path.dsv
cat > "$HOOK_DIR/ament_prefix_path.dsv" << 'EOF'
prepend-non-duplicate;AMENT_PREFIX_PATH;
EOF

# Create ament_prefix_path.sh
cat > "$HOOK_DIR/ament_prefix_path.sh" << 'EOF'
# copied from colcon_core/shell/template/prefix_util.sh.em
_colcon_prefix_sh_ament_prepend_unique_value() {
  # arguments
  _listname="$1"
  _value="$2"
  #echo "listname $_listname"
  #eval echo "list value \$$_listname"
  #echo "value $_value"

  # check if the list contains the value
  eval _values=\"\$$_listname\"
  _duplicate=
  _colcon_prefix_sh_prepend_unique_value_IFS=$IFS
  IFS=":"
  if [ "$AMENT_SHELL" = "zsh" ]; then
    ament_zsh_to_array _values
  fi
  for _item in $_values; do
    # ignore empty strings
    if [ -z "$_item" ]; then
      continue
    fi
    if [ $_item = $_value ]; then
      _duplicate=1
    fi
  done
  unset _item

  # append only non-duplicates
  if [ -z "$_duplicate" ]; then
    # avoid leading separator
    if [ -z "$_values" ]; then
      eval export $_listname=\"$_value\"
      #eval echo "set list \$$_listname"
    else
      # field separator must not be a colon
      unset IFS
      eval export $_listname=\"$_value:\$$_listname\"
      #eval echo "prepend list \$$_listname"
    fi
  fi
  IFS=$_colcon_prefix_sh_prepend_unique_value_IFS
  unset _colcon_prefix_sh_prepend_unique_value_IFS
  unset _duplicate
  unset _values

  unset _value
  unset _listname
}

# function to convert array-like strings into arrays
# to wordaround SH_WORD_SPLIT not being set in zsh
if [ "$AMENT_SHELL" = "zsh" ]; then
  ament_zsh_to_array() {
    _listname=$1
    eval _dollar_ref=\$${_listname}
    _str="$_dollar_ref"
    _list="$(echo "$_str" | sed "s/:/\\n/g")"
    eval $_listname="(${_list})"
  }
fi

# prepend AMENT_PREFIX_PATH with this directory
_colcon_prefix_sh_ament_prepend_unique_value AMENT_PREFIX_PATH "$COLCON_CURRENT_PREFIX"

unset _colcon_prefix_sh_ament_prepend_unique_value
unset ament_zsh_to_array
EOF

# Create ament_prefix_path.ps1
cat > "$HOOK_DIR/ament_prefix_path.ps1" << 'EOF'
# generated from colcon_powershell/shell/template/prefix_util.ps1.em

function colcon_prepend_unique_value {
  param (
    $_listname,
    $_value
  )

  $ErrorActionPreference = "SilentlyContinue"

  # get values from the environment variable
  if (Test-Path Env:$_listname) {
    $_values=(Get-Item Env:$_listname).Value
  } else {
    $_values=""
  }
  # backup the field separator
  $_colcon_prepend_unique_value_IFS=$FS
  # start with the new value
  $_all_values=@($_value)
  # iterate over existing values in the variable
  if ($_values) {
    $_values.Split(";") | ForEach {
      # ignore empty strings
      if ($_) {
        # ignore duplicates of _value
        if ($_ -eq $_value) {
          # continue
        } else {
          $_all_values+=$_
        }
      }
    }
  }
  # export the updated variable
  Set-Item Env:$_listname -Value ($_all_values -join ";")
}

# prepend AMENT_PREFIX_PATH with this directory
colcon_prepend_unique_value AMENT_PREFIX_PATH "$env:COLCON_CURRENT_PREFIX"

Remove-Item Function:colcon_prepend_unique_value
EOF

# Update package.dsv to include ament_prefix_path hooks
cat > "$PACKAGE_DSV" << 'EOF'
source;share/voice_llm_navigator/hook/ament_prefix_path.ps1
source;share/voice_llm_navigator/hook/ament_prefix_path.dsv
source;share/voice_llm_navigator/hook/ament_prefix_path.sh
source;share/voice_llm_navigator/hook/pythonpath.ps1
source;share/voice_llm_navigator/hook/pythonpath.dsv
source;share/voice_llm_navigator/hook/pythonpath.sh
EOF

echo "✓ Ament hooks installed for $PACKAGE_NAME"
