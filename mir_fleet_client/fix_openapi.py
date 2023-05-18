# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Fixes up the openapi schema so the generated code works.

import json
import os

import yaml

here = os.path.dirname(__file__)

with open(f"{here}/openapi.json") as resp:
    openapi = json.loads(resp.read())

# mir wrongly includes the authorization header to each endpoints, this will cause problems
# in the generated code so we remove them.
print("Removing authorization header in endpoints")
if "parameters" in openapi and "authorization" in openapi["parameters"]:
    del openapi["parameters"]["authorization"]
for ep in openapi["paths"].values():
    for meth in ep.values():
        for idx, param in enumerate(meth["parameters"]):
            if "$ref" in param and param["$ref"] == "#/parameters/authorization":
                meth["parameters"].pop(idx)
                break

# it is also missing the auth info
print("Adding security definitions")
if "security" not in openapi:
    openapi["security"] = [{"Basic": []}]

with open(f"{here}/openapi.yaml", "w") as f:
    yaml.safe_dump(openapi, f)
print("Openapi schema fixed and converted to yaml")
