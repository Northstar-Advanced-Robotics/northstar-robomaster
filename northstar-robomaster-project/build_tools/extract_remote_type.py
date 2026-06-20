from SCons.Script import *

DEFAULT_REMOTE = "DJI_REMOTE"

VALID_REMOTE_TYPES   = [ "DJI_REMOTE", 
                        "FLY_SKY"]

def get_remote_type():
    remote_type = ARGUMENTS.get("remote")
    # Configure robot type and check against valid robot type
    # If there is no optional argument, revert back to the macro in robot_type.hpp
    if remote_type == None:
        remote_type = DEFAULT_REMOTE
    if remote_type not in VALID_REMOTE_TYPES:
        raise Exception("No valid remote type specified. check extract_remote_type.py for valid remote types")

    return remote_type
