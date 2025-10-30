#!/bin/bash
# Fix all ROS 2 Python launchers to use venv Python
VENV_PYTHON="/home/akari/aura_ws/.venv/bin/python"
INSTALL_DIR="/home/akari/aura_ws/install"

echo "🔧 Fixing ROS Python launchers to use $VENV_PYTHON ..."

# Find all Python entry points (executables) inside install/
find "$INSTALL_DIR" -type f -executable \
    -not -path "*/share/*" \
    -exec grep -Il "^#!" {} \; | while read -r file; do
    if head -n 1 "$file" | grep -q "python"; then
        sed -i "1s|.*|#!$VENV_PYTHON|" "$file"
        echo "  ✅ Patched: $file"
    fi
done

echo "✨ Done! All ROS Python nodes now use your venv."
