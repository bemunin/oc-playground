#!/bin/bash

NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_foundationpose"
NGC_RESOURCE="isaac_ros_foundationpose_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1

# Define the workspace directory
if [ -z "$ISAAC_ROS_WS" ]; then
    echo "Environment variable ISAAC_ROS_WS is not set."
    exit 1
fi

VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"

# Fetch available versions
AVAILABLE_VERSIONS=$(curl -s -H "Accept: application/json" "$VERSION_REQ_URL")
if [ $? -ne 0 ] || [ -z "$AVAILABLE_VERSIONS" ]; then
    echo "Failed to fetch versions from NGC. Please check your network or the URL."
    exit 1
fi

# Debugging: Print the raw JSON response
echo "Raw JSON response:"
echo "$AVAILABLE_VERSIONS"

# Validate JSON format
if ! echo "$AVAILABLE_VERSIONS" | jq empty > /dev/null 2>&1; then
    echo "Error: Invalid JSON received from the server."
    exit 1
fi

# Extract the latest version ID
LATEST_VERSION_ID=$(echo "$AVAILABLE_VERSIONS" | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
" | sort -V | tail -n 1)

if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo "$AVAILABLE_VERSIONS" | jq -r '.recipeVersions[].versionId'
    exit 1
fi

# Download and extract the latest version
mkdir -p "${ISAAC_ROS_WS}/isaac_ros_assets"
FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$LATEST_VERSION_ID/files/$NGC_FILENAME"

curl -LO --request GET "${FILE_REQ_URL}"
if [ $? -ne 0 ]; then
    echo "Failed to download the file: $NGC_FILENAME"
    exit 1
fi

tar -xf "$NGC_FILENAME" -C "${ISAAC_ROS_WS}/isaac_ros_assets"
if [ $? -ne 0 ]; then
    echo "Failed to extract the file: $NGC_FILENAME"
    exit 1
fi

rm "$NGC_FILENAME"
echo "Assets successfully downloaded and extracted to ${ISAAC_ROS_WS}/isaac_ros_assets."

