#!/bin/bash
set -e

cd "$(dirname "$0")"
if [[ ! -f build/openapi-generator-cli-6.1.0.jar ]]; then
  mkdir -p build
  pushd build
  curl -LO https://repo1.maven.org/maven2/org/openapitools/openapi-generator-cli/6.1.0/openapi-generator-cli-6.1.0.jar
  popd
fi

java -jar build/openapi-generator-cli-6.1.0.jar generate -g python -c openapi_config.yaml -i "openapi.yaml" --skip-validate-spec

# fix __init__.py to export the modified `Configuration`
sed -i 's/from mir_fleet_client.configuration import Configuration/from mir_fleet_client.mir_configuration import MirConfiguration as Configuration/' mir_fleet_client/__init__.py
