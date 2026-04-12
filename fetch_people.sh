#!/bin/bash
# Downloads NVIDIA Isaac Sim People/Characters assets from NVIDIA's public S3.
# Assets are covered by the NVIDIA Isaac Sim Additional Software and Materials License.
# Each user downloads directly from NVIDIA's servers — this is NOT redistribution.
# License: https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/

set -e
set -o pipefail

BASE_URL="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/People/Characters"
DEST_DIR="assets/humans"

# Characters to download (name : folder on S3)
declare -A CHARACTERS=(
    ["male_construction"]="male_adult_construction_01_new"
    ["female_business"]="F_Business_02"
    ["female_medical"]="F_Medical_01"
)

mkdir -p "$DEST_DIR"

download_character() {
    local name=$1
    local folder=$2
    local dest="$DEST_DIR/$name"

    echo "Downloading character: $name ($folder)..."
    mkdir -p "$dest/textures"

    # Download main USD
    curl -fsSL "$BASE_URL/$folder/$folder.usd" -o "$dest/$name.usd"

    # Download textures (BaseColor, Normal, ORM)
    for tex in $(curl -s "https://omniverse-content-production.s3-us-west-2.amazonaws.com/?prefix=Assets/Isaac/4.2/Isaac/People/Characters/$folder/textures/" \
                 | grep -o "<Key>[^<]*\.png</Key>" | sed 's/<Key>//;s/<\/Key>//' | grep -v thumb); do
        filename=$(basename "$tex")
        curl -fsSL "http://omniverse-content-production.s3-us-west-2.amazonaws.com/$tex" -o "$dest/textures/$filename"
        echo "  ✓ $filename"
    done

    echo "✅ $name done → $dest/"
}

for name in "${!CHARACTERS[@]}"; do
    folder="${CHARACTERS[$name]}"
    if [ -f "$DEST_DIR/$name/$name.usd" ]; then
        echo "⏭  $name already exists, skipping."
    else
        download_character "$name" "$folder"
    fi
done

echo ""
echo "✅ All people assets ready in $DEST_DIR/"
