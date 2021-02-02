#!/usr/bin/env python3
""" Manage git subtree dependencies. """

import argparse
import collections
import configparser
import os
import os.path
import subprocess
import sys

Dependency = collections.namedtuple("Dependency", ["directory", "repository", "default_branch"])


def _read_dependencies(dependencies_file):
    dependencies = {}
    if not os.path.exists(dependencies_file):
        sys.stderr.write("cannot find dependency file %s\n" % dependencies_file)
    else:
        config = configparser.ConfigParser()
        config.read(dependencies_file)
        for section_name in config.sections():
            dependencies[section_name] = Dependency(
                config.get(section_name, "path_in_repo"),
                config.get(section_name, "upstream_url"),
                config.get(section_name, "upstream_ref"),
            )
    return dependencies


def init_dependency(dep_name, branch_name, squash, url_override):
    """Initialize dependency with git subtree add"""
    dep_dir = PROJECT_DEPENDENCIES[dep_name].directory
    if url_override is not None:
        url = url_override
    else:
        url = PROJECT_DEPENDENCIES[dep_name].repository
    if not os.path.isdir(dep_dir):
        # assume that we have to initialize it
        parent_dir = os.path.dirname(dep_dir)
        if not os.path.isdir(parent_dir):
            os.makedirs(parent_dir)
        if branch_name is None:
            branch_name = PROJECT_DEPENDENCIES[dep_name].default_branch
        # relay work to git
        subtree_add_cmd = " ".join(["git subtree add", "--prefix", dep_dir, url, branch_name])
        if squash:
            subtree_add_cmd += " --squash"
        print("executing %s" % subtree_add_cmd)
        return subprocess.call(subtree_add_cmd, shell=True)
    else:
        return 0  # success


def pull_dependency(dep_name, branch_name, squash, url_override):
    """Update dependency with git subtree pull"""
    if branch_name is None:
        branch_name = PROJECT_DEPENDENCIES[dep_name].default_branch
    if url_override is not None:
        url = url_override
    else:
        url = PROJECT_DEPENDENCIES[dep_name].repository
    # relay work to git
    subtree_pull_cmd = " ".join(
        ["git subtree pull", "--prefix", PROJECT_DEPENDENCIES[dep_name].directory, url, branch_name]
    )
    if squash:
        subtree_pull_cmd += " --squash"
    print("executing %s" % subtree_pull_cmd)
    return subprocess.call(subtree_pull_cmd, shell=True)


def push_dependency(dep_name, branch_name, url_override):
    """Push local changes in dependency to origin repository"""
    if branch_name is None:
        print(
            "Please specify an upstream branch name with -b.\nDefault branch for this repository during update is %s"
            % PROJECT_DEPENDENCIES[dep_name].default_branch
        )
        exit(1)
    if url_override is not None:
        url = url_override
    else:
        url = PROJECT_DEPENDENCIES[dep_name].repository
    subtree_push_cmd = " ".join(
        ["git subtree push", "--prefix", PROJECT_DEPENDENCIES[dep_name].directory, url, branch_name]
    )
    print("executing %s" % subtree_push_cmd)
    return subprocess.call(subtree_push_cmd, shell=True)


DEPENDENCIES_FILENAME = "repo_dependencies.ini"
PROJECT_DEPENDENCIES = _read_dependencies(DEPENDENCIES_FILENAME)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="update code of dependencies managed by git subtree; location and default branch of dependencies is managed within this script"
    )
    parser.add_argument(
        "operation",
        choices=["init", "pull", "push"],
        help="operation to be performed (analogous to git subtree add/pull/push)",
    )
    parser.add_argument("dep_name", choices=sorted(list(PROJECT_DEPENDENCIES.keys())), help="dependency to update")
    parser.add_argument("-b", "--branch", help="remote ref (branch or tag name) to use")
    parser.add_argument(
        "-s",
        "--squash",
        type=bool,
        default=True,
        help="squash commits (combine all dependency commits into a single new one); default true",
    )
    parser.add_argument(
        "-u",
        "--url",
        help="URL to upstream repo (for overriding information from %s, should probably be used in conjunction with -b)"
        % DEPENDENCIES_FILENAME,
    )

    args = parser.parse_args()

    if args.operation == "init":
        ret_code = init_dependency(args.dep_name, args.branch, args.squash, args.url)
    elif args.operation == "pull":
        ret_code = pull_dependency(args.dep_name, args.branch, args.squash, args.url)
    elif args.operation == "push":
        ret_code = push_dependency(args.dep_name, args.branch, args.url)
    else:
        print("unknown command")
        exit(1)
    exit(ret_code)
