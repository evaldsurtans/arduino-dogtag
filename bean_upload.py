#!/bin/python

import sys

from SCons.Script import DefaultEnvironment
env = DefaultEnvironment()

def before_upload(source, target, env):
    cmd = "bean program_sketch %s -n %s" % (source[0], env['UPLOAD_PORT'])
    env.Execute(cmd)
    sys.exit(0)
env.AddPreAction("upload", before_upload)
