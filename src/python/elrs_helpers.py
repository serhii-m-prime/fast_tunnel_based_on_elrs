import os
import re
import subprocess

def git_cmd(*args):
    return subprocess.check_output(["git"] + list(args), stderr=subprocess.DEVNULL).decode("utf-8").rstrip('\r\n')


def get_git_version():
    """
    Return a dict with keys
    version: The version tag if HEAD is a version, or branch otherwise
    sha: the 6 character short sha for the current HEAD revison, falling back to
        VERSION file if not in a git repo
    """
    ver = "ver.unknown"
    sha = "000000"

    try:
        sha = git_cmd("rev-parse", "HEAD")
        ver = git_cmd("rev-parse", "--abbrev-ref", "HEAD")
        # failure here is acceptable, unnamed commits might not have a branch
        # associated
        try:
            ver = re.sub(r".*/", "", git_cmd("describe",
                         "--all", "--exact-match"))
        except:
            pass
    except:
        if os.path.exists("VERSION"):
            with open("VERSION") as _f:
                data = _f.readline()
                _f.close()
            sha = data.split()[1].strip()

    return dict(version=ver, sha=sha[:6])


def get_target_type(target_name, build_flags=()):
    """
    Determine target role ('TX' or 'RX') from build flags or PIOENV target name.
    Supports both standard Unified_*_(TX|RX)_* names and custom board environments
    such as HappyModel_ES900TX_Bridge / HappyModel_ES900RX_Bridge.
    """
    for flag in (build_flags or ()):
        for token in str(flag).split():
            if token.startswith("-DTARGET_RX"):
                return "RX"
            if token.startswith("-DTARGET_TX"):
                return "TX"

    if not target_name:
        return None

    base_name = re.sub(r"_VIA_.*$", "", str(target_name).upper())
    for part in base_name.split("_"):
        if part == "RX" or part.endswith("RX"):
            return "RX"
        if part == "TX" or part.endswith("TX"):
            return "TX"
    return None


class ElrsUploadResult:
        # SUCCESS
        Success = 0
        # ERROR: Unspecified
        ErrorGeneral = -1
        # ERROR: target mismatch
        ErrorMismatch = -2
