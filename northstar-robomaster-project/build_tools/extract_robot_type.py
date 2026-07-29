# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of aruw-mcb.
#
# aruw-mcb is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# aruw-mcb is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.

from build_tools.parse_args import USAGE
from SCons.Script import *

# TODO: Make this sync up with check.py and c_cpp_properties.json if possible
# Each robot type must have a matching directory under src/robot/ and a branch
# in the target conditionals (robot_control.hpp, drivers_singleton.hpp, ...).
VALID_ROBOT_TYPES   = [ "STANDARD",
                        "HERO",
                        "SENTRY",
                        "TURRET",
                        "TEST_BED"]


ROBOT_CLASS = {
    "STANDARD": "standard",
    "TURRET": "turret",
    "SENTRY": "sentry",
    "HERO": "hero",
    "TEST_BED": "testbed",
}

# Make sure that all robots have a class
assert all([robot in ROBOT_CLASS.keys() for robot in VALID_ROBOT_TYPES])


def search_for_robot_type(query):
    if not query:
        return []
    # Accept the documented "TARGET_<ROBOT_TYPE>" form as well as bare names
    if query.upper().startswith("TARGET_"):
        query = query[len("TARGET_"):]
    # An exact match always wins, even if it is also a substring of another type
    exact = [robot for robot in VALID_ROBOT_TYPES if query.lower() == robot.lower()]
    if exact:
        return exact
    return [robot for robot in VALID_ROBOT_TYPES if query.lower() in robot.lower()]


def get_robot_type():
    robot_query = ARGUMENTS.get("robot")
    robot_type_matches = search_for_robot_type(robot_query)

    if len(robot_type_matches) != 1:
        if not robot_query or not robot_type_matches:
            prompt = "Please enter a valid robot type out of the following:\n"
        else:
            prompt = "Robot type is ambiguous, please enter a valid robot type or unique substring out of the following:\n"

        for type in VALID_ROBOT_TYPES:
            prompt += type + "\n"
        prompt += "--> "
        try:
            robot_query = input(prompt)
        except EOFError:
            # Non-interactive build (CI, scripts): fail with a helpful message
            # instead of an EOFError traceback.
            raise Exception(
                "No valid robot type specified and cannot prompt for one "
                "(stdin is not interactive). Pass robot=<ROBOT_TYPE>.\n" + USAGE
            )
        robot_type_matches = search_for_robot_type(robot_query)

    # Check against valid robot type
    if len(robot_type_matches) != 1:
        raise Exception(USAGE)

    return "TARGET_" + robot_type_matches[0]
