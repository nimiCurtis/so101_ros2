import os
import sys


def ensure_conda_site_packages_from_env():
    conda_site = os.environ.get("LECONDA_SITE_PACKAGES")
    if conda_site and conda_site not in sys.path:
        sys.path.insert(0, conda_site)
        print(f"✅ Added {conda_site} to sys.path")
    else:
        print(f"ℹ️ {conda_site} already in sys.path or not set.")


# def ensure_conda_site_packages_from_env(var_name="LECONDA_SITE_PACKAGES"):
#     """
#     Ensures the Conda site-packages path defined in an environment variable
#     is added to sys.path. Raises helpful errors if not set or invalid.

#     Args:
#         var_name (str): Name of the environment variable holding the site-packages path.
#     """
#     conda_site = os.environ.get(var_name)
#     if not conda_site:
#         raise EnvironmentError(f"{var_name} environment variable is not set.")

#     if not os.path.exists(conda_site):
#         raise EnvironmentError(f"{var_name} path '{conda_site}' does not exist.")

#     if conda_site not in sys.path:
#         sys.path.append(conda_site)
#         print(f"Added {conda_site} to sys.path")
#     else:
#         print(f"{conda_site} is already in sys.path")
