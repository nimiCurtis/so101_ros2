# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os
import sys


def ensure_conda_site_packages_from_env(var_name="LECONDA_SITE_PACKAGES"):
    """
    Ensures the Conda site-packages path defined in an environment variable
    is added to sys.path. Raises helpful errors if not set or invalid.

    Args:
        var_name (str): Name of the environment variable holding the site-packages path.
    """
    conda_site = os.environ.get(var_name)
    if not conda_site:
        raise EnvironmentError(f"{var_name} environment variable is not set.")

    if not os.path.exists(conda_site):
        raise EnvironmentError(f"{var_name} path '{conda_site}' does not exist.")

    if conda_site not in sys.path:
        sys.path.append(conda_site)
        print(f"Added {conda_site} to sys.path")
    else:
        print(f"{conda_site} is already in sys.path")
