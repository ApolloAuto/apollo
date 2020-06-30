"""Loads the adv_plat library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "adv_plat",
        build_file = clean_dep("//third_party/adv_plat:adv_plat.BUILD"),
        path = "/opt/apollo/pkgs/adv_plat/include",
    )
