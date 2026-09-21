# Copyright 2026 stevej52
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

"""Skip the tests whose dependencies are not installed."""

import importlib.util

collect_ignore = []
if importlib.util.find_spec('rclpy') is None:
    collect_ignore.append('test_drive_node.py')
for linter in ('copyright', 'flake8', 'pep257'):
    if importlib.util.find_spec(f'ament_{linter}') is None:
        collect_ignore.append(f'test_{linter}.py')
